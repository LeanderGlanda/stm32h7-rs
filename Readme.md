# Setting up development environment

Make sure rustup is installed and then run

`rustup target add thumbv7em-none-eabihf`

I think you also need the drivers for ST-link as well, at least I installed them. 
Get them from the ST website:
[STM32H755 dev board download page](https://www.st.com/en/evaluation-tools/nucleo-h755zi-q.html#tools-software)

And for flashing to the board, you'll need probe-rs too. [probe-rs download page](https://probe.rs/docs/getting-started/installation/)

Run the project using `cargo run`

The basic structure of this project was taken from embassy-stm32 examples.