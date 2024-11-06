#!/bin/bash

sudo modprobe -r xhci_pci

sleep 10

sudo modprobe xhci_pci

