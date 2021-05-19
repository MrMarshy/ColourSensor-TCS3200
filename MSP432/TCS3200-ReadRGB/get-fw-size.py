import subprocess
import codecs
import os
import argparse




def print_region(binary, max_flash, max_ram):
    
    print(f"BINARY: {binary}")
    print(f"MAX FLASH: {max_flash}")
    print(f"MAX RAM: {max_ram}")

    
    raw = subprocess.check_output(["arm-none-eabi-size", binary])
    
    print(raw.decode('ascii'))
    
    _text = raw.decode('ascii').split('\r\n')[1].strip().split('\t')[0].strip() # text
    _data = raw.decode('ascii').split('\r\n')[1].strip().split('\t')[1].strip() # data
    _bss = raw.decode('ascii').split('\r\n')[1].strip().split('\t')[2].strip() # bss
    _dec = raw.decode('ascii').split('\r\n')[1].strip().split('\t')[3].strip() # dec
    _hex = raw.decode('ascii').split('\r\n')[1].strip().split('\t')[4].strip() # hex
    _file =  raw.decode('ascii').split('\r\n')[1].strip().split('\t')[5].strip() # file

    flash = int(_text) + int(_data)
    ram = int(_data) + int(_bss)

    print(f"Flash used: {flash} / {int(max_flash, 16)} ({(100 * flash / int(max_flash, 16)):.2f}%)")
    print(f"Ram used: {ram} / {int(max_ram, 16)} ({(100 * ram / int(max_ram, 16)):.2f}%)")


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(dest="binary", type=str, help="The path to the binary file to calculate firmware size")
    parser.add_argument(dest="max_flash", type=str, help="The max size in hex of the flash (hint: check linker script)")
    parser.add_argument(dest="max_ram", type=str, help="The max size in hex of the ram (hint: check linker script")

    args = parser.parse_args()

    max_flash = hex(int(args.max_flash[2:], 16))
    max_ram = hex(int(args.max_ram[2:], 16))

    print_region(args.binary, max_flash, max_ram)

