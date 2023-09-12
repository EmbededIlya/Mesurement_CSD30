 //! The following wiring is assumed:
//! - SDA => GPIO1
//! - SCL => GPIO2
// We output data on acceleration along the x axis and temperature

#![no_std]
#![no_main]

// Libraries
use esp32c3_hal::{
    clock::ClockControl,
    interrupt,
    peripherals::{self, Peripherals, TIMG0, TIMG1,I2C0},
    prelude::*,
    gpio::{ Gpio5, Output, PushPull,IO},
    riscv,
    i2c::I2C,
    timer::{Timer, Timer0, TimerGroup},
    Rtc,
    Delay,
};
use esp_backtrace as _;
use esp_println::println;
use core::cell::RefCell;
use critical_section::Mutex;

// Static variables
static TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));
static TIMER1: Mutex<RefCell<Option<Timer<Timer0<TIMG1>>>>> = Mutex::new(RefCell::new(None));
static I_2_C: Mutex<RefCell<Option<I2C<I2C0>>>> = Mutex::new(RefCell::new(None));

//main
#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    // Begining of the project
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    println!("      (Project is begining!!!)"); 

    // Create a new peripheral object with the described wiring
    // and standard I2C clock speed
    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        50u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    // Enable interrapts
    interrupt::enable(
        peripherals::Interrupt::TG0_T0_LEVEL,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    //Init timer
    let mut timer0 = timer_group0.timer0;

    // Start timer on 10 sec
    timer0.start(3000u64.millis());
    timer0.listen();
    critical_section::with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
    });
    
    // Write comands of the sensor SCD30
    let mut cmnd_trigger_continuous: [u8;6] = [0xC2, 0, 0x10, 0, 0, 0x81];
    let mut cmnd_set_interval: [u8;6] = [0xC2,0x46, 0, 0,0x02,0xE3];
    let mut  exp=   [0u8;10];
    i2c.write_read(0x61, &[ 0xD1,0x00], &mut  exp);
    
    

    // Trigger continuous measurement with optional ambient pressure compensation
    i2c.write(0x61, &cmnd_trigger_continuous);

    // Set measurement interval
    i2c.write(0x61, &cmnd_set_interval);

    // Place the I2C to global  variable
    critical_section::with(|cs| I_2_C.borrow_ref_mut(cs).replace(i2c));

    // Unsafe
    unsafe {
        riscv::interrupt::enable();
    }
    // Loop
    loop {
        
    }
}

#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        
        esp_println::println!("Interrupt");

        // Enabling the delay function
       /*  let peripherals = Peripherals::take();
        let mut system = peripherals.SYSTEM.split();
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
        let mut delay = Delay::new(&clocks); */

        // Unnpackage the timer
        let mut timer0 = TIMER0.borrow_ref_mut(cs);
        let timer0 = timer0.as_mut().unwrap();

        // The sensor data
        let mut CO2: f32;
        let mut temperature =  0.0;
        let mut humidity =  0.0; 

        // Write comands of the sensor SCD30
        let cmnd_get_redy_status: [u8;3] = [0xC2, 0x02, 0x02]; // Note that the read header should be send with a delay of > 3ms following the write sequence.
        let cmnd_read_mesurment: [u8;3] = [0xC2, 0x03, 0x00]; //Note that the read header should be send with a delay of > 3ms following the write sequence

        // Arrays of unsigned numbers 1 byte for data
        let mut data = [0u8; 13];
        let mut status = [0u8; 3];

        // Unnpackage the i2c
        let mut i2c = I_2_C.borrow_ref_mut(cs);
        let mut i2c = i2c.as_mut().unwrap();

        // Requesting the status of data
        i2c.write(0x61, &cmnd_get_redy_status);
        let mut i   = 0;
        loop{
            if(i==2000000)
            {
                break;
            }
            i=  i+1;
        }
        /* delay.delay_ms(4u32); */
        i2c.read(0x61, &mut status).ok();
        println!("    Status = {}, {}, {}",status[0],status[1],status[2]); 
        // Receive the data
        //if status[2] == 1 {
            // Read measurement 
            i2c.write(0x61, &cmnd_read_mesurment);
           /*  delay.delay_ms(4u32);
         */
            // Requesting and reading data 
            i2c.read(0x61, &mut data).ok(); 

            // Parsing CO2  data  
            CO2 = parsing_data(&data[1], &data[2], &data[3], &data[4] );

            // Parsing temperature  data  
            temperature = parsing_data(&data[5], &data[6], &data[7], &data[8] );

            // Parsing humidity  data  
            humidity = parsing_data(&data[9], &data[10], &data[11], &data[12] );

            // Data output to the console
            print_data( &CO2, &temperature, &humidity) ;
        //}

        //Restart 
        timer0.clear_interrupt();
        timer0.start(3000u64.millis());
    });
}



// Functions
fn parsing_data( mmsb: &u8, mlsb: &u8, lmsb: &u8, llsb: &u8 ) -> f32 {

    // Init parts of the message
    let zero_byte: i32 ;  
    let first_byte: i32 ; 
    let second_byte: i32 ;  
    let third_byte: i32 ; 
    
    //
    zero_byte = (*mmsb as i32) << 24;
    first_byte = (*mlsb as i32) << 16; 
    second_byte = (*lmsb as i32) << 8;
    third_byte = *llsb as i32; 
    
    // Return
    (zero_byte | first_byte | second_byte | third_byte) as f32 

}


fn print_data( co2: &f32, temperature: &f32, humidity: &f32) {
    // Data output to the console
    println!("  Measurement:"); 
    println!("    CO2 = {} ",co2); 
    println!("    Temperature = {} ",temperature); 
    println!("    Humidity = {} ",humidity); 
}

