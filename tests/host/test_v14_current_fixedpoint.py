#!/usr/bin/env python3
"""Host arithmetic checks for V14 ADC-current fixed-point scaling.
This mirrors the integer path in foc_control.c without needing the STM32 HAL.
"""
BASE_A=64.0
A_PER_COUNT=0.0200
scale_q16=int((A_PER_COUNT/BASE_A)*32768.0*65536.0)

def adc_to_q15(raw, offset):
    return ((offset-raw)*scale_q16) >> 16

def q15_to_a(q):
    return q*BASE_A/32768.0

assert scale_q16 == 671088, scale_q16
for counts, expected in [(1,0.02),(50,1.0),(250,5.0),(500,10.0),(1250,25.0),(-50,-1.0)]:
    raw=2000-counts
    q=adc_to_q15(raw,2000)
    got=q15_to_a(q)
    # Absolute error stays below one Q15 LSB plus current ADC quantization fraction.
    assert abs(got-expected) < 0.0021, (counts,q,got,expected)

# Kirchhoff reconstruction paths used by the two-shunt board.
ia=adc_to_q15(1950,2000); ib=adc_to_q15(2025,2000); ic=-(ia+ib)
assert ia+ib+ic == 0
ib_r=adc_to_q15(1940,2000); ic_r=adc_to_q15(2040,2000); ia_r=-(ib_r+ic_r)
assert ia_r+ib_r+ic_r == 0

# 25 A software trip should map to exactly 12800 Q15 units at a 64 A base.
trip=int((25.0/BASE_A)*32768.0)
assert trip == 12800
print('test_v14_current_fixedpoint: PASS')
print('  scale_q16=',scale_q16)
print('  50 ADC counts ->',q15_to_a(adc_to_q15(1950,2000)),'A')
print('  1250 ADC counts ->',q15_to_a(adc_to_q15(750,2000)),'A')
