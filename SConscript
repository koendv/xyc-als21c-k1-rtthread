# SConscript file for rt-thread
import os
import rtconfig
from building import *

cwd     = GetCurrentDir()
src     = Glob('/src/*.c')
path    = [cwd + '/inc']

group = DefineGroup('xyc-asl21c-k1', src, depend = ['PKG_USING_XYC_ALS21C_K1'], CPPPATH = path)

Return('group')
