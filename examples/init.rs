use std::env;
use std::process;
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
    // let blocks = 8;

    let mut buffer: Dma<u8> = Dma::allocate(HUGE_PAGE_SIZE, true)?;
    let n = "hello world".len();
    buffer[..n].copy_from_slice(b"hello world");
    nvme.write(&buffer.slice(0..n), 0)?;

    buffer[..n].fill_with(Default::default);
    nvme.read(&buffer.slice(0..n), 0)?;

    println!("{}", std::str::from_utf8(&buffer[..n])?);

    /*
    let n = 10;
    let n2 = 1000;
    let mut read = std::time::Duration::new(0, 0);
    let mut write = std::time::Duration::new(0, 0);

    // let mut rng = rand::thread_rng();
    // use rand::seq::SliceRandom;
    let mut seq: Vec<u64> = Vec::from_iter(0..n);

    for _ in 0..n2 {
        // seq.shuffle(&mut rng);
        let lba = 0;
        for i in &seq {
            let rand_block = &(0..(512 * blocks))
                .map(|_| rand::random::<u8>())
                .collect::<Vec<_>>()[..];

            buffer[..rand_block.len()].copy_from_slice(rand_block);

            // write
            let before = std::time::Instant::now();
            nvme.write(&buffer, lba + (*i * blocks as u64))?;
            write += before.elapsed();

            buffer[..rand_block.len()].fill_with(Default::default);
            let before = std::time::Instant::now();
            nvme.read(
                1,
                &buffer,
                lba + (*i * blocks as u64),
            )?;
            read += before.elapsed();

            assert_eq!(&buffer[..rand_block.len()], rand_block);
            //  lba += blocks as u64;
        }
    }

    println!("total completions: {}", nvme.stats.completions);
    println!("total submissions: {}", nvme.stats.submissions);
    println!(
        "read time: {:?}; write time: {:?}; total: {:?}",
        read,
        write,
        read + write
    );
    */

    Ok(())
}
