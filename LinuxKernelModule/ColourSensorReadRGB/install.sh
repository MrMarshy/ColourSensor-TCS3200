#!/bin/bash

sudo insmod coloursensor.ko
dmesg | tail -n 10
