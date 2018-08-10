#!/bin/bash

echo "Building testit container..."
cd $(rospack find testit_dtron)/docker
docker build --no-cache -t testit_dtron:latest .
