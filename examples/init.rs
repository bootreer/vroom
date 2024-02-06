use std::env;
use std::process;

pub fn main() {
    let mut args = env::args();
    args.next();

    let pci_addr = match args.next() {
        Some(arg) => arg,
        None => {
            eprintln!("Usage: cargo run --example init <pci bus id>");
            process::exit(1);
        }
    };
    let _dev = vroom::init(&pci_addr).unwrap();
}
