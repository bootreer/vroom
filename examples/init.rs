use std::env;
use std::process;
// use std::sync::{Mutex, Arc};
use vroom::QUEUE_LENGTH;
use vroom::memory::*;

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args();
    args.next();

    let pci_addr = match args.next() {
        Some(arg) => arg,
        None => {
            eprintln!("Usage: cargo run --example init <pci bus id>");
            process::exit(1);
        }
    };

    let mut nvme = vroom::init(&pci_addr)?;
    nvme.create_io_queue_pair(QUEUE_LENGTH)?;


    // Testing stuff
    let blocks = 8;
    let bytes = 512 * blocks;
    let ns_blocks = nvme.namespaces.get(&1).unwrap().blocks - blocks - 1;

    let mut buffer: Dma<u8> = Dma::allocate(HUGE_PAGE_SIZE, true)?;

    let n = 100_000;
    let mut read = std::time::Duration::new(0, 0);
    let mut write = std::time::Duration::new(0, 0);

    // let nvme = Arc::new(Mutex::new(nvme));

    use rand::{thread_rng, Rng};
    let mut rng = thread_rng();
    for _ in 0..n {
        let lba = rng.gen_range(0..ns_blocks);
        let rand_block = &(0..bytes)
            .map(|_| rand::random::<u8>())
            .collect::<Vec<_>>()[..];

        buffer[..rand_block.len()].copy_from_slice(rand_block);

        // write
        let before = std::time::Instant::now();
        nvme.write(&buffer.slice(0..bytes as usize), lba)?;
        write += before.elapsed();

        buffer[..rand_block.len()].fill_with(Default::default);
        let before = std::time::Instant::now();
        nvme.read(
            &buffer.slice(0..bytes as usize),
            lba ,
        )?;
        read += before.elapsed();

        assert_eq!(&buffer[0..rand_block.len()], rand_block);
        // lba += blocks as u64;
    }

    println!("total completions: {}", nvme.stats.completions);
    println!("total submissions: {}", nvme.stats.submissions);
    println!(
        "read time: {:?}; write time: {:?}; total: {:?}",
        read,
        write,
        read + write
    );

    Ok(())
}
