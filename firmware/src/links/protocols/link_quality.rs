use embassy_futures::select::{Either, select};
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Receiver;
use embassy_time::{Duration, Instant, Ticker, Timer};
use rapid_dialect::rapid::messages::LinkNodeStatus;
use static_cell::StaticCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use rapid_dialect::rapid::{
    enums::{MavModeProperty, MavStandardMode},
    messages::AvailableModes,
};
use rapid_dialect::{FlightMode, Rapid};

use crate::links::UplinkCommand;
use crate::links::interfaces::{
    InterfaceCommandSubscriber, InterfaceRxSubscriber, InterfaceTxPublisher,
};

#[derive(Clone, Default)]
pub struct LinkQuality {
    pub tx_rate: u32,
    pub rx_rate: u32,
    pub messages_received: u32,
    pub messages_lost: u32,
}

#[embassy_executor::task(pool_size = 2)]
pub async fn run(
    tx: InterfaceTxPublisher,
    mut rx: Receiver<'static, CriticalSectionRawMutex, LinkQuality, 3>,
) {
    let mut link_quality = LinkQuality::default();
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let t = Instant::now();

    loop {
        match select(ticker.next(), rx.changed()).await {
            Either::First(()) => {
                tx.publish(Rapid::LinkNodeStatus(LinkNodeStatus {
                    timestamp: t.elapsed().as_millis(),
                    tx_buf: 100,
                    rx_buf: 100,
                    tx_rate: link_quality.tx_rate,
                    rx_rate: link_quality.rx_rate,
                    rx_parse_err: 0,
                    tx_overflows: 0,
                    rx_overflows: 0,
                    messages_sent: 0,
                    messages_received: link_quality.messages_received,
                    messages_lost: link_quality.messages_lost,
                }))
                .await;
            }
            Either::Second(lq) => {
                link_quality = lq;
            }
        }
    }
}
