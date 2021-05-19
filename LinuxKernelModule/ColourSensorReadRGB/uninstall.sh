#!/bin/bash

sudo rmmod coloursensor.ko
dmesg | tail -n 10

