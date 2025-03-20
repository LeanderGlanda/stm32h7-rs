#![no_std]
#![no_main]

use core::mem::MaybeUninit;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::eth::generic_smi::GenericSMI;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::ETH;
use embassy_stm32::rng::Rng;
use embassy_stm32::{bind_interrupts, eth, SharedData};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};
use embassy_stm32::eth::Ethernet;
use embassy_stm32::eth::PacketQueue;


use core::net::{Ipv4Addr, SocketAddr, SocketAddrV4};

use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::StackResources;
use embassy_stm32::rng;
use embedded_io_async::Write;
use embedded_nal_async::TcpConnect;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};



bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
    HASH_RNG => rng::InterruptHandler<embassy_stm32::peripherals::RNG>;
});


#[unsafe(link_section = ".ram_d3.shared_data")]
static SHARED_DATA: MaybeUninit<SharedData> = MaybeUninit::uninit();


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8), // 100mhz
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.supply_config = SupplyConfig::DirectSMPS;
    }
    let p = embassy_stm32::init_primary(config, &SHARED_DATA);
    info!("Hello World!");

    let mut led = Output::new(p.PB14, Level::High, Speed::Low);


    // Generate random seed.
    let mut rng = Rng::new(p.RNG, Irqs);
    let mut seed = [0; 8];
    rng.async_fill_bytes(&mut seed).await.unwrap();
    let seed = u64::from_le_bytes(seed);

    let mac_addr = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();

    let device = Ethernet::new(
        PACKETS.init(PacketQueue::<4, 4>::new()),
        p.ETH,
        Irqs,
        p.PA1,
        p.PA2,
        p.PC1,
        p.PA7,
        p.PC4,
        p.PC5,
        p.PG13,
        p.PB13,
        p.PG11,
        GenericSMI::new(0),
        mac_addr,
    );

    unwrap!(spawner.spawn(ethernet_task(spawner, device, seed)));


    loop {
        // info!("high");
        led.set_high();
        Timer::after_millis(500).await;

        // info!("low");
        led.set_low();
        Timer::after_millis(500).await;
    }
}



type Device = Ethernet<'static, ETH, GenericSMI>;

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}


#[embassy_executor::task]
async fn ethernet_task(spawner: Spawner, device: Device, seed: u64) {

    info!("Running network task");
    

    let config = embassy_net::Config::dhcpv4(Default::default());
    //let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
    //    address: Ipv4Cidr::new(Ipv4Address::new(10, 42, 0, 61), 24),
    //    dns_servers: Vec::new(),
    //    gateway: Some(Ipv4Address::new(10, 42, 0, 1)),
    //});

    // Init network stack
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(device, config, RESOURCES.init(StackResources::new()), seed);

    // Launch network task
    unwrap!(spawner.spawn(net_task(runner)));

    // Ensure DHCP configuration is up before trying connect
    stack.wait_config_up().await;

    info!("Network task initialized");

    let state: TcpClientState<1, 1024, 1024> = TcpClientState::new();
    let client = TcpClient::new(stack, &state);

    loop {
        // You need to start a server on the host machine, for example: `nc -l 8000`
        let addr = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(192, 168, 178, 28), 8000));

        info!("connecting...");
        let r = client.connect(addr).await;
        if let Err(e) = r {
            info!("connect error: {:?}", e);
            Timer::after_secs(1).await;
            continue;
        }
        let mut connection = r.unwrap();
        info!("connected!");
        loop {
            let r = connection.write_all(b"Hello\n").await;
            if let Err(e) = r {
                info!("write error: {:?}", e);
                break;
            }
            Timer::after_secs(1).await;
        }
    }
}