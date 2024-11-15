fn main() {
    /*
     * TODO: MAVLink handling
     */

    /*
     * Generate Rust-to-C bindings (allow C to call to Rust code)
     */

    cbindgen::Builder::new()
        .with_src("src/main.rs")
        .with_language(cbindgen::Language::C)
        .with_cpp_compat(true)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("Core/Inc/rust_bindings.h");

    /*
     * Invoke CMake for STM32 C code.
     */

    let dst: std::path::PathBuf = cmake::Config::new(".").generator("Ninja").build();

    /*
     * Link the C code static library.
     */

    println!("cargo::rustc-link-search=native={}/lib", dst.display());
    println!("cargo::rustc-link-lib=c-code");

    /*
     * Linker arguments. (copy-pasted from cmake/gcc-arm-none-eabi.cmake)
     */

    let pkg_name = std::env::var("CARGO_PKG_NAME").unwrap();
    println!("cargo::rustc-link-arg=-mcpu=cortex-m7");
    println!("cargo::rustc-link-arg=-mfpu=fpv5-d16");
    println!("cargo::rustc-link-arg=-mfloat-abi=hard");
    println!("cargo::rustc-link-arg=-Wl,-Map={}.map", pkg_name);
    println!("cargo::rustc-link-arg=-TSTM32H753IITx_FLASH.ld");
    println!("cargo::rustc-link-arg=--specs=nano.specs");
    println!("cargo::rustc-link-arg=-Wl,--gc-sections");
    println!("cargo::rustc-link-arg=-Wl,--start-group");
    println!("cargo::rustc-link-arg=-lc");
    println!("cargo::rustc-link-arg=-lm");
    println!("cargo::rustc-link-arg=-Wl,--end-group");
    println!("cargo::rustc-link-arg=-Wl,--print-memory-usage");

    /* Copy compile_commands.json to root directory for clangd VSCode extension */
    let mut compile_commands_path = dst.clone();
    compile_commands_path.push("build");
    compile_commands_path.push("compile_commands.json");
    std::fs::copy(compile_commands_path, "compile_commands.json").unwrap();
}
