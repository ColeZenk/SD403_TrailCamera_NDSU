---
geometry: margin=0.5in
---
# TODO


## LoRa Field Deployment
- [ ] Solder 100-470 uF cap across RYLR998 VCC/GND on S3 PCB (Bug 5 — SDD004)
- [ ] Re-test AT+CRFOP=22 after cap installed
- [ ] Switch bench config to BW=7 (125 kHz) for field range — currently BW=9 (500 kHz)
- [ ] Walk-test at real distance: 50 m, 500 m, 1 mile, 2 miles
- [ ] Tune BENCH_INTER_PACKET_MS to reduce half-duplex collisions (~50% loss observed)
- [ ] Clean up debug logging (ESP_LOGW beacon counter) in S3 lora_bench.c
- [ ] Consolidate lora_bench.c and lora_uart.c into shared LoRa interface

## WHT Compression
- [ ] FPGA WHT butterfly engine — pipeline integration (wht8.v exists, not integrated)
- [ ] Coefficient extraction and patch-based compression
- [ ] DevKitV1 compressed TX — encode coefficients into LoRa packets
- [ ] S3 reconstruction from WHT coefficients

## Web App / Mobile
- [ ] Flutter app — WebSocket client connecting to S3 AP (ws://192.168.4.1:80/stream)
- [ ] Frame display — render reconstructed grayscale frames from binary WS messages
- [ ] Bare bones settings manager (SF, BW, TX power, packet count)

## Display / Settings
- [ ] TFT display on FPGA — status, config, live preview
- [ ] Settings interface — runtime RF parameter adjustment without reflash

## Integration
- [ ] Disable TEST_MODE_LORA_BENCH on both boards when moving to production
- [ ] End-to-end pipeline: camera capture -> WHT compress -> LoRa TX -> reconstruct -> display
- [ ] Power budget verification for field deployment
