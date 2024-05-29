use std::error::Error;
use std::{env, process};

pub fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args();
    args.next();

    let pci_addr = match args.next() {
        Some(arg) => arg,
        None => {
            eprintln!("Usage: cargo run --example hello_world <pci bus id>");
            process::exit(1);
        }
    };

    let mut nvme = vroom::init(&pci_addr)?;
    nvme.write_copied("hello world".as_bytes(), 0)?;

    let mut dest = [0u8; 12];
    nvme.read_copied(&mut dest, 0)?;

    println!("{}", std::str::from_utf8(&dest)?);

    Ok(())
}
