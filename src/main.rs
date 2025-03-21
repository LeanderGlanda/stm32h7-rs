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

use grounded::uninit::GroundedArrayCell;
use embassy_stm32::sai::*;
use embassy_stm32::time::Hertz;



bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
    HASH_RNG => rng::InterruptHandler<embassy_stm32::peripherals::RNG>;
});

const BLOCK_LENGTH: usize = 32; // 32 samples
const HALF_DMA_BUFFER_LENGTH: usize = BLOCK_LENGTH * 2; //  2 channels
const DMA_BUFFER_LENGTH: usize = HALF_DMA_BUFFER_LENGTH * 2; //  2 half-blocks
const SAMPLE_RATE: u32 = 48000;

//DMA buffer must be in special region. Refer https://embassy.dev/book/#_stm32_bdma_only_working_out_of_some_ram_regions
#[unsafe(link_section = ".sram1_bss")]
static mut TX_BUFFER: GroundedArrayCell<u32, DMA_BUFFER_LENGTH> = GroundedArrayCell::uninit();
#[unsafe(link_section = ".sram1_bss")]
static mut RX_BUFFER: GroundedArrayCell<u32, DMA_BUFFER_LENGTH> = GroundedArrayCell::uninit();



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
        config.rcc.pll3 = Some(Pll {
            source: PllSource::HSE, // Nucleo-144 uses a 8Mhz HSE clock input
            prediv: PllPreDiv::DIV3, 
            mul: PllMul::MUL295,
            divp: Some(PllDiv::DIV16),
            divq: Some(PllDiv::DIV4),
            divr: Some(PllDiv::DIV32),
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.mux.sai1sel = embassy_stm32::pac::rcc::vals::Saisel::PLL3_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.supply_config = SupplyConfig::DirectSMPS;
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(8),
            mode: HseMode::Bypass,
        });
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



    // I2S stuff:

    let (sub_block_tx, sub_block_rx) = embassy_stm32::sai::split_subblocks(p.SAI1);
    let kernel_clock = embassy_stm32::rcc::frequency::<embassy_stm32::peripherals::SAI1>().0;
    let mclk_div = mclk_div_from_u8((kernel_clock / (SAMPLE_RATE * 256)) as u8);

    let mut tx_config = embassy_stm32::sai::Config::default();
    tx_config.mode = Mode::Master;
    tx_config.tx_rx = TxRx::Transmitter;
    tx_config.sync_output = true;
    tx_config.clock_strobe = ClockStrobe::Falling;
    tx_config.master_clock_divider = mclk_div;
    tx_config.stereo_mono = StereoMono::Stereo;
    tx_config.data_size = DataSize::Data24;
    tx_config.bit_order = BitOrder::MsbFirst;
    tx_config.frame_sync_polarity = FrameSyncPolarity::ActiveHigh;
    tx_config.frame_sync_offset = FrameSyncOffset::OnFirstBit;
    tx_config.frame_length = 64;
    tx_config.frame_sync_active_level_length = embassy_stm32::sai::word::U7(32);
    tx_config.fifo_threshold = FifoThreshold::Quarter;

    let mut rx_config = tx_config.clone();
    rx_config.mode = Mode::Slave;
    rx_config.tx_rx = TxRx::Receiver;
    rx_config.sync_input = SyncInput::Internal;
    rx_config.clock_strobe = ClockStrobe::Rising;
    rx_config.sync_output = false;

    let tx_buffer: &mut [u32] = unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(TX_BUFFER);
        buf.initialize_all_copied(0);
        let (ptr, len) = buf.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let sai_transmitter = Sai::new_asynchronous_with_mclk(
        sub_block_tx,
        p.PE5,
        p.PE6,
        p.PE4,
        p.PE2,
        p.DMA1_CH0,
        tx_buffer,
        tx_config,
    );

    let rx_buffer: &mut [u32] = unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(RX_BUFFER);
        buf.initialize_all_copied(0);
        let (ptr, len) = buf.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let mut sai_receiver = Sai::new_synchronous(sub_block_rx, p.PE3, p.DMA1_CH1, rx_buffer, rx_config);

    sai_receiver.start().unwrap();


    unwrap!(spawner.spawn(sai_task(sai_transmitter, sai_receiver)));
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
type SaiTR = Sai<'static, embassy_stm32::peripherals::SAI1, u32>;

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn sai_task(mut sai_transmitter: SaiTR, mut sai_receiver: SaiTR ) {

    let mut buf= [0u32; HALF_DMA_BUFFER_LENGTH];

    loop {
        sai_receiver.read(&mut buf).await.unwrap();
        sai_transmitter.write(&buf).await.unwrap();
    }
}



/// Note: use something like "ncat -l -p 8000" on Windows to receive the packets
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


const fn mclk_div_from_u8(v: u8) -> MasterClockDivider {
    match v {
        1 => MasterClockDivider::Div1,
        2 => MasterClockDivider::Div2,
        3 => MasterClockDivider::Div3,
        4 => MasterClockDivider::Div4,
        5 => MasterClockDivider::Div5,
        6 => MasterClockDivider::Div6,
        7 => MasterClockDivider::Div7,
        8 => MasterClockDivider::Div8,
        9 => MasterClockDivider::Div9,
        10 => MasterClockDivider::Div10,
        11 => MasterClockDivider::Div11,
        12 => MasterClockDivider::Div12,
        13 => MasterClockDivider::Div13,
        14 => MasterClockDivider::Div14,
        15 => MasterClockDivider::Div15,
        16 => MasterClockDivider::Div16,
        17 => MasterClockDivider::Div17,
        18 => MasterClockDivider::Div18,
        19 => MasterClockDivider::Div19,
        20 => MasterClockDivider::Div20,
        21 => MasterClockDivider::Div21,
        22 => MasterClockDivider::Div22,
        23 => MasterClockDivider::Div23,
        24 => MasterClockDivider::Div24,
        25 => MasterClockDivider::Div25,
        26 => MasterClockDivider::Div26,
        27 => MasterClockDivider::Div27,
        28 => MasterClockDivider::Div28,
        29 => MasterClockDivider::Div29,
        30 => MasterClockDivider::Div30,
        31 => MasterClockDivider::Div31,
        32 => MasterClockDivider::Div32,
        33 => MasterClockDivider::Div33,
        34 => MasterClockDivider::Div34,
        35 => MasterClockDivider::Div35,
        36 => MasterClockDivider::Div36,
        37 => MasterClockDivider::Div37,
        38 => MasterClockDivider::Div38,
        39 => MasterClockDivider::Div39,
        40 => MasterClockDivider::Div40,
        41 => MasterClockDivider::Div41,
        42 => MasterClockDivider::Div42,
        43 => MasterClockDivider::Div43,
        44 => MasterClockDivider::Div44,
        45 => MasterClockDivider::Div45,
        46 => MasterClockDivider::Div46,
        47 => MasterClockDivider::Div47,
        48 => MasterClockDivider::Div48,
        49 => MasterClockDivider::Div49,
        50 => MasterClockDivider::Div50,
        51 => MasterClockDivider::Div51,
        52 => MasterClockDivider::Div52,
        53 => MasterClockDivider::Div53,
        54 => MasterClockDivider::Div54,
        55 => MasterClockDivider::Div55,
        56 => MasterClockDivider::Div56,
        57 => MasterClockDivider::Div57,
        58 => MasterClockDivider::Div58,
        59 => MasterClockDivider::Div59,
        60 => MasterClockDivider::Div60,
        61 => MasterClockDivider::Div61,
        62 => MasterClockDivider::Div62,
        63 => MasterClockDivider::Div63,
        _ => core::panic!(),
    }
}
