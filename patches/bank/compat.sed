# API differences between the feature branch (current master) and the pinned
# 4.5.1, applied by scripts/bank-sync.sh to the staged copy of bank_mode.c.
# Master: acc.accADC is a vector3_t. 4.5.1: float accADC[XYZ_AXIS_COUNT].
# Same value either way, rotated but unscaled counts; acc_1G_rec scales it.
s/acc\.accADC\.x/acc.accADC[X]/g
s/acc\.accADC\.y/acc.accADC[Y]/g
s/acc\.accADC\.z/acc.accADC[Z]/g
