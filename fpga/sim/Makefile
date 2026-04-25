RTL = ../src/rtl
IV = iverilog
VVP = vvp

all: wht8 coeff esp_tx blk_schd

wht8: wht8_tb.v $(RTL)/wht8.v
	$(IV) -o $@.vvp $^
	$(VVP) $@.vvp

coeff: coeff_tb.v $(RTL)/coeff.v
	$(IV) -o $@.vvp $^
	$(VVP) $@.vvp

esp_rx: esp_interface_tb.v $(RTL)/esp_interface.v
	$(IV) -o $@.vvp $^
	$(VVP) $@.vvp

esp_tx: esp_interface_tx_tb.v $(RTL)/esp_interface.v
	$(IV) -o $@.vvp $^
	$(VVP) $@.vvp

blk_schd: blk_schd_tb.v $(RTL)/blk_schd.v $(RTL)/wht8.v $(RTL)/wht2d.v \
           $(RTL)/seq_msk.v $(RTL)/coeff.v
	$(IV) -o $@.vvp $^
	$(VVP) $@.vvp

clean:
	rm -f *.vvp *.vcd

.PHONY: all clean wht8 coeff esp_rx esp_tx blk_schd
