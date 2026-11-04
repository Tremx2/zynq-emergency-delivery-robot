## =========================================
## PYNQ-Z2 四电机引脚约束 (修正版)
## PWM 与编码器两组引脚已按实车接线对调:
##   AR0~AR7   : 四电机 PWM (RPWM/LPWM -> BTS7960 驱动板)
##   AR8~AR13  : 电机0~2 编码器 C1/C2 (经电平转换 3.3V 端)
##   PMODA 1/2 : 电机3 编码器 C1/C2
## =========================================

## =========================================
## 四电机 PWM (到 BTS7960 驱动板)
## =========================================

## 电机 0 PWM
## RPWM -> AR1
set_property PACKAGE_PIN U12 [get_ports {rpwm_0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpwm_0[0]}]

## LPWM -> AR0
set_property PACKAGE_PIN T14 [get_ports {lpwm_0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {lpwm_0[0]}]

## 电机 1 PWM
## RPWM -> AR3
set_property PACKAGE_PIN V13 [get_ports {rpwm_0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpwm_0[1]}]

## LPWM -> AR2
set_property PACKAGE_PIN U13 [get_ports {lpwm_0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {lpwm_0[1]}]

## 电机 2 PWM
## RPWM -> AR5
set_property PACKAGE_PIN T15 [get_ports {rpwm_0[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpwm_0[2]}]

## LPWM -> AR4
set_property PACKAGE_PIN V15 [get_ports {lpwm_0[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {lpwm_0[2]}]

## 电机 3 PWM
## RPWM -> AR7
set_property PACKAGE_PIN U17 [get_ports {rpwm_0[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rpwm_0[3]}]

## LPWM -> AR6
set_property PACKAGE_PIN R16 [get_ports {lpwm_0[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {lpwm_0[3]}]

## =========================================
## 四电机编码器 C1/C2 (经电平转换 3.3V 端)
## =========================================

## 电机 0 编码器
## C1 -> AR8
set_property PACKAGE_PIN V17 [get_ports {enc_a_0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enc_a_0[0]}]

## C2 -> AR9
set_property PACKAGE_PIN V18 [get_ports {enc_b_0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enc_b_0[0]}]

## 电机 1 编码器
## C1 -> AR10
set_property PACKAGE_PIN T16 [get_ports {enc_a_0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enc_a_0[1]}]

## C2 -> AR11
set_property PACKAGE_PIN R17 [get_ports {enc_b_0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enc_b_0[1]}]

## 电机 2 编码器
## C1 -> AR12
set_property PACKAGE_PIN P18 [get_ports {enc_a_0[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enc_a_0[2]}]

## C2 -> AR13
set_property PACKAGE_PIN N17 [get_ports {enc_b_0[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enc_b_0[2]}]

## 电机 3 编码器
## C1 -> PMODA 1
set_property PACKAGE_PIN Y18 [get_ports {enc_a_0[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enc_a_0[3]}]

## C2 -> PMODA 2
set_property PACKAGE_PIN Y19 [get_ports {enc_b_0[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enc_b_0[3]}]

## =========================================
## 板上 I2C (SCL/SDA, 与电机无关)
## =========================================
set_property PACKAGE_PIN T11 [get_ports {iic_rtl_0_sda_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {iic_rtl_0_sda_io}]
set_property PULLUP true [get_ports {iic_rtl_0_sda_io}]

set_property PACKAGE_PIN T10 [get_ports {iic_rtl_0_scl_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {iic_rtl_0_scl_io}]
set_property PULLUP true [get_ports {iic_rtl_0_scl_io}]

## =========================================
## TOF XSHUT (PMOD B pin 1 = W14, 单 ToF)
## =========================================
set_property PACKAGE_PIN W14 [get_ports {tof_xshut_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {tof_xshut_0}]

## 3 位改 1 位: Y14/W13 已移除
