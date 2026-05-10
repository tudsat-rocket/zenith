if ip link show tap99 >/dev/null 2>&1; then
    exit 0
fi

sudo ip tuntap add name tap99 mode tap user $USER
sudo ip link set tap99 up
sudo ip addr add 192.168.69.100/24 dev tap99
sudo ip -6 addr add fe80::100/64 dev tap99
sudo ip -6 addr add fdaa::100/64 dev tap99
sudo ip -6 route add fe80::/64 dev tap99
sudo ip -6 route add fdaa::/64 dev tap99
