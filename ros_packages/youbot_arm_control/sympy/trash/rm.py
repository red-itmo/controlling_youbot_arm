#!/usr/bin/env python3
import xi_00
import time

if __name__ == '__main__':
    xi = xi_00.XI()
    t = time.time()
    x = xi.getXI((1,1,1,1,1),(1,1,10,1,10),(0,0,1,0,0),(10,20,10,0,0),(10,0,0,0,30))
    print((time.time() - t)/60*1000)
    print(x)
