//! Derive macros for the parameter system.
//!
//! - `#[derive(ParameterGroup)]` on a struct whose fields are annotated with
//!   `#[param(id = .., name = "..", default = ..)]` generates a `params::ParameterGroup` impl plus
//!   a `Default` impl built from the declared defaults. Optionally, `unit = ".."`, `min = ..` and
//!   `max = ..` describe the value, and `reboot_required` marks a param that only takes effect on
//!   the next boot. The first paragraph of each field's doc comment is the description ground
//!   stations show. The struct-level `#[param_group(prefix = "..")]` supplies the MAVLink name
//!   prefix (the first underscored segment).
//! - `#[derive(ParameterGroups)]` on an aggregate struct whose fields are themselves parameter
//!   groups delegates to each sub-group.

use proc_macro::TokenStream;
use proc_macro2::{Span, TokenStream as TokenStream2};
use quote::quote;
use syn::punctuated::Punctuated;
use syn::spanned::Spanned;
use syn::token::Comma;
use syn::{
    Attribute, Data, DataStruct, DeriveInput, Expr, ExprLit, Field, Fields, Lit, LitStr, Meta,
    MetaNameValue, RangeLimits, Type, parse_macro_input,
};

/// Lengths of the `description` and `units` fields of PARAM_METADATA.
const DESCRIPTION_LEN: usize = 200;
const UNIT_LEN: usize = 16;

#[proc_macro_derive(ParameterGroup, attributes(param_group, param))]
pub fn derive_parameter_group(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    expand_group(&input)
        .unwrap_or_else(syn::Error::into_compile_error)
        .into()
}

#[proc_macro_derive(ParameterGroups, attributes(param_skip))]
pub fn derive_parameter_groups(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    expand_groups(&input)
        .unwrap_or_else(syn::Error::into_compile_error)
        .into()
}

fn named_fields(input: &DeriveInput) -> syn::Result<&Punctuated<Field, Comma>> {
    match &input.data {
        Data::Struct(DataStruct {
            fields: Fields::Named(fields),
            ..
        }) => Ok(&fields.named),
        _ => Err(syn::Error::new_spanned(
            &input.ident,
            "this derive only supports structs with named fields",
        )),
    }
}

fn parse_prefix(attrs: &[Attribute]) -> syn::Result<String> {
    for attr in attrs {
        if attr.path().is_ident("param_group") {
            let mut prefix = None;
            attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("prefix") {
                    let lit: LitStr = meta.value()?.parse()?;
                    prefix = Some(lit.value());
                    Ok(())
                } else {
                    Err(meta.error("unknown #[param_group] argument (expected `prefix`)"))
                }
            })?;
            return prefix.ok_or_else(|| {
                syn::Error::new_spanned(attr, "#[param_group] requires `prefix = \"..\"`")
            });
        }
    }
    Err(syn::Error::new(
        Span::call_site(),
        "missing #[param_group(prefix = \"..\")] on the struct",
    ))
}

struct ParamAttr {
    ids: Vec<u16>,
    name: String,
    name_span: Span,
    defaults: Vec<Expr>,
    unit: String,
    min: Option<Expr>,
    max: Option<Expr>,
    reboot_required: bool,
    description: String,
    span: Span,
}

impl ParamAttr {
    /// The declared bounds of a field of type `field_ty`, or its scalar's own.
    fn bounds(&self, field_ty: &Type) -> (TokenStream2, TokenStream2) {
        let scalar =
            quote! { <<#field_ty as ::params::ParameterField>::Scalar as ::params::ParamScalar> };
        let bound = |declared: &Option<Expr>, fallback| {
            declared
                .as_ref()
                .map_or_else(|| quote! { #scalar::#fallback }, |bound| quote! { #bound })
        };
        (
            bound(&self.min, quote! { MIN }),
            bound(&self.max, quote! { MAX }),
        )
    }

    /// The `descriptor()` match arm of the slot at flat `index` with `id` and full `name`.
    fn descriptor_arm(
        &self,
        field_ty: &Type,
        index: usize,
        id: u16,
        name: &LitStr,
    ) -> TokenStream2 {
        let scalar =
            quote! { <<#field_ty as ::params::ParameterField>::Scalar as ::params::ParamScalar> };
        let (min, max) = self.bounds(field_ty);
        let description = &self.description;
        let unit = &self.unit;
        let reboot_required = self.reboot_required;
        quote! {
            #index => ::core::option::Option::Some(::params::ParamDescriptor {
                id: ::params::ParamId::new(#id),
                name: #name,
                ty: <#field_ty as ::params::ParameterField>::SLOT_TYPE,
                description: #description,
                unit: #unit,
                min: #scalar::value(#min),
                max: #scalar::value(#max),
                reboot_required: #reboot_required,
            }),
        }
    }

    /// Compile-time checks of the declaration against a field of type `field_ty`.
    fn assertions(&self, field_ty: &Type) -> TokenStream2 {
        let width = self.ids.len();
        let mut assertions = quote! {
            const _: () = ::core::assert!(
                #width == <#field_ty as ::params::ParameterField>::WIDTH,
                "the number of declared ids must match the field's parameter width"
            );
        };
        if self.min.is_some() || self.max.is_some() {
            let (min, max) = self.bounds(field_ty);
            let defaults = &self.defaults;
            assertions.extend(quote! {
                const _: () = {
                    let min: <#field_ty as ::params::ParameterField>::Scalar = #min;
                    let max: <#field_ty as ::params::ParameterField>::Scalar = #max;
                    #(
                        let default: <#field_ty as ::params::ParameterField>::Scalar = #defaults;
                        ::core::assert!(
                            min <= default && default <= max,
                            "a parameter's default must lie within its bounds"
                        );
                    )*
                };
            });
        }
        assertions
    }
}

fn parse_param(field: &Field) -> syn::Result<ParamAttr> {
    let attr = field
        .attrs
        .iter()
        .find(|a| a.path().is_ident("param"))
        .ok_or_else(|| {
            syn::Error::new_spanned(
                field,
                "field needs #[param(id = .., name = \"..\", default = ..)]",
            )
        })?;

    let mut id_expr = None;
    let mut name = None;
    let mut default = None;
    let mut unit = None;
    let mut min = None;
    let mut max = None;
    let mut reboot_required = false;
    attr.parse_nested_meta(|meta| {
        if meta.path.is_ident("id") {
            id_expr = Some(meta.value()?.parse::<Expr>()?);
        } else if meta.path.is_ident("name") {
            name = Some(meta.value()?.parse::<LitStr>()?);
        } else if meta.path.is_ident("default") {
            default = Some(meta.value()?.parse::<Expr>()?);
        } else if meta.path.is_ident("unit") {
            unit = Some(meta.value()?.parse::<LitStr>()?);
        } else if meta.path.is_ident("min") {
            min = Some(meta.value()?.parse::<Expr>()?);
        } else if meta.path.is_ident("max") {
            max = Some(meta.value()?.parse::<Expr>()?);
        } else if meta.path.is_ident("reboot_required") {
            reboot_required = true;
        } else {
            return Err(meta.error(
                "unknown #[param] argument \
                 (expected id, name, default, unit, min, max, reboot_required)",
            ));
        }
        Ok(())
    })?;

    let id_expr = id_expr.ok_or_else(|| syn::Error::new_spanned(attr, "#[param] missing `id`"))?;
    let name = name.ok_or_else(|| syn::Error::new_spanned(attr, "#[param] missing `name`"))?;
    let default =
        default.ok_or_else(|| syn::Error::new_spanned(attr, "#[param] missing `default`"))?;

    let unit = match unit {
        Some(lit) if lit.value().len() > UNIT_LEN || !lit.value().is_ascii() => {
            return Err(syn::Error::new_spanned(
                lit,
                format!("a unit is at most {UNIT_LEN} ASCII characters"),
            ));
        }
        Some(lit) => lit.value(),
        None => String::new(),
    };

    Ok(ParamAttr {
        ids: parse_ids(&id_expr)?,
        name: name.value(),
        name_span: name.span(),
        defaults: parse_defaults(default),
        unit,
        min,
        max,
        reboot_required,
        description: parse_description(field)?,
        span: attr.span(),
    })
}

/// The first paragraph of a field's doc comment, joined into one line.
fn parse_description(field: &Field) -> syn::Result<String> {
    let docs = field.attrs.iter().filter_map(|attr| match &attr.meta {
        Meta::NameValue(MetaNameValue {
            path,
            value: Expr::Lit(ExprLit {
                lit: Lit::Str(doc), ..
            }),
            ..
        }) if path.is_ident("doc") => Some(doc.value()),
        _ => None,
    });

    let mut lines = Vec::new();
    for doc in docs {
        for line in doc.lines().map(str::trim) {
            if !line.is_empty() {
                lines.push(line.to_string());
            } else if !lines.is_empty() {
                return check_description(field, lines.join(" "));
            }
        }
    }
    check_description(field, lines.join(" "))
}

fn check_description(field: &Field, description: String) -> syn::Result<String> {
    if description.is_empty() {
        Err(syn::Error::new_spanned(
            field,
            "a parameter needs a doc comment, its first paragraph is shown by ground stations",
        ))
    } else if !description.is_ascii() || description.len() > DESCRIPTION_LEN {
        Err(syn::Error::new_spanned(
            field,
            format!(
                "the first paragraph of a parameter's doc comment must be at most \
                 {DESCRIPTION_LEN} ASCII characters, this one is {}",
                description.len()
            ),
        ))
    } else {
        Ok(description)
    }
}

fn expr_to_u16(expr: &Expr) -> syn::Result<u16> {
    if let Expr::Lit(ExprLit {
        lit: Lit::Int(int), ..
    }) = expr
    {
        int.base10_parse::<u16>()
    } else {
        Err(syn::Error::new_spanned(
            expr,
            "expected an integer literal id",
        ))
    }
}

fn parse_ids(expr: &Expr) -> syn::Result<Vec<u16>> {
    match expr {
        Expr::Range(range) => {
            let start = range
                .start
                .as_deref()
                .ok_or_else(|| syn::Error::new_spanned(range, "id range needs a start"))?;
            let end = range
                .end
                .as_deref()
                .ok_or_else(|| syn::Error::new_spanned(range, "id range needs an end"))?;
            let start = expr_to_u16(start)?;
            let end = expr_to_u16(end)?;
            let ids: Vec<u16> = match range.limits {
                RangeLimits::Closed(_) => (start..=end).collect(),
                RangeLimits::HalfOpen(_) => (start..end).collect(),
            };
            if ids.is_empty() {
                return Err(syn::Error::new_spanned(range, "empty id range"));
            }
            Ok(ids)
        }
        _ => Ok(vec![expr_to_u16(expr)?]),
    }
}

fn parse_defaults(expr: Expr) -> Vec<Expr> {
    match expr {
        Expr::Array(array) => array.elems.into_iter().collect(),
        other => vec![other],
    }
}

/// Arrays are numbered, other composites (vectors) get their axis names.
fn slot_suffix(index: usize, width: usize, array: bool) -> &'static str {
    if width == 1 {
        return "";
    }
    match (array, index) {
        (true, 0) => "_0",
        (true, 1) => "_1",
        (true, 2) => "_2",
        (true, _) => "_3",
        (false, 0) => "_X",
        (false, 1) => "_Y",
        (false, 2) => "_Z",
        (false, _) => "_W",
    }
}

#[allow(
    clippy::expect_used,
    reason = "named_fields() rejects tuple/unit structs, so every field ident is Some"
)]
fn expand_group(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let ident = &input.ident;
    let prefix = parse_prefix(&input.attrs)?;
    let fields = named_fields(input)?;

    let mut descriptors = Vec::new();
    let mut get_arms = Vec::new();
    let mut set_arms = Vec::new();
    let mut default_inits = Vec::new();
    let mut assertions = Vec::new();
    let mut total = 0usize;

    for field in fields {
        let field_ident = field.ident.as_ref().expect("named field");
        let field_ty = &field.ty;
        let param = parse_param(field)?;
        let width = param.ids.len();

        if width > 4 {
            return Err(syn::Error::new(
                param.span,
                "composite fields wider than 4 slots are not supported yet",
            ));
        }
        if param.defaults.len() != width {
            return Err(syn::Error::new(
                param.span,
                format!("expected {width} default value(s) to match the id count"),
            ));
        }

        for (slot, id) in param.ids.iter().enumerate() {
            let array = matches!(field_ty, Type::Array(_));
            let full = format!("{prefix}_{}{}", param.name, slot_suffix(slot, width, array));
            if full.len() > 16 {
                return Err(syn::Error::new(
                    param.name_span,
                    format!("parameter name \"{full}\" exceeds the 16-char MAVLink limit"),
                ));
            }
            let name_lit = LitStr::new(&full, param.name_span);
            let id = *id;
            let index = total
                .checked_add(slot)
                .ok_or_else(|| syn::Error::new(param.span, "too many parameters"))?;
            descriptors.push(param.descriptor_arm(field_ty, index, id, &name_lit));
            get_arms.push(quote! {
                #id => ::core::option::Option::Some(
                    ::params::ParameterField::get_slot(&self.#field_ident, #slot)
                ),
            });
            set_arms.push(quote! {
                #id => ::params::ParameterField::set_slot(&mut self.#field_ident, #slot, value),
            });
        }

        let defaults = &param.defaults;
        default_inits.push(quote! {
            #field_ident: <#field_ty as ::params::ParameterField>::from_slots(&[#(#defaults),*]),
        });
        assertions.push(param.assertions(field_ty));
        total = total
            .checked_add(width)
            .ok_or_else(|| syn::Error::new(param.span, "too many parameters"))?;
    }

    Ok(quote! {
        #(#assertions)*

        impl ::params::ParameterGroup for #ident {
            const PARAM_COUNT: usize = #total;

            fn descriptor(index: usize) -> ::core::option::Option<::params::ParamDescriptor> {
                match index {
                    #(#descriptors)*
                    _ => ::core::option::Option::None,
                }
            }

            fn get(&self, id: ::params::ParamId) -> ::core::option::Option<::params::ParamValue> {
                match id.get() {
                    #(#get_arms)*
                    _ => ::core::option::Option::None,
                }
            }

            fn set(&mut self, id: ::params::ParamId, value: ::params::ParamValue) -> bool {
                match id.get() {
                    #(#set_arms)*
                    _ => false,
                }
            }
        }

        impl ::core::default::Default for #ident {
            fn default() -> Self {
                Self { #(#default_inits)* }
            }
        }
    })
}

fn has_skip(field: &Field) -> bool {
    field.attrs.iter().any(|a| a.path().is_ident("param_skip"))
}

#[allow(
    clippy::expect_used,
    reason = "named_fields() rejects tuple/unit structs, so every field ident is Some"
)]
fn expand_groups(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let ident = &input.ident;
    let fields = named_fields(input)?;

    let groups: Vec<&Field> = fields.iter().filter(|f| !has_skip(f)).collect();
    let idents: Vec<_> = groups
        .iter()
        .map(|f| f.ident.as_ref().expect("named field"))
        .collect();
    let tys: Vec<_> = groups.iter().map(|f| &f.ty).collect();

    Ok(quote! {
        impl ::params::ParameterGroup for #ident {
            const PARAM_COUNT: usize = 0usize #( + <#tys as ::params::ParameterGroup>::PARAM_COUNT )*;

            #[allow(
                unused_assignments,
                clippy::arithmetic_side_effects,
                reason = "flat-index routing over sub-group counts, bounded by PARAM_COUNT"
            )]
            fn descriptor(index: usize) -> ::core::option::Option<::params::ParamDescriptor> {
                let mut offset = 0usize;
                #(
                    {
                        const COUNT: usize = <#tys as ::params::ParameterGroup>::PARAM_COUNT;
                        if index < offset + COUNT {
                            return <#tys as ::params::ParameterGroup>::descriptor(index - offset);
                        }
                        offset += COUNT;
                    }
                )*
                ::core::option::Option::None
            }

            fn get(&self, id: ::params::ParamId) -> ::core::option::Option<::params::ParamValue> {
                ::core::option::Option::None
                #( .or_else(|| ::params::ParameterGroup::get(&self.#idents, id)) )*
            }

            fn set(&mut self, id: ::params::ParamId, value: ::params::ParamValue) -> bool {
                #(
                    if ::params::ParameterGroup::set(&mut self.#idents, id, value) {
                        return true;
                    }
                )*
                false
            }
        }
    })
}
