from ros import UrdfClass
from Other import load_json, save_json, clock, Concepts, MyCsv, get_key, listener, pickle_load_data, pickle_save_data
from scipy.spatial import distance
import numpy as np
import copy
from datetime import datetime, timedelta
from tqdm import tqdm
from time import time, sleep
import os
import shutil
import sys
import getpass
from multiprocessing import Process
from simulator import simulate
# from ros_error_check import listener
# import shlex
# import subprocess
# from ros import Ros


# np.random.seed(100100)
# # np.random.seed(1010101)
# # np.random.seed(111111)
# # np.random.seed(0)
from optimization import Optimization


gen_num = 1240
start_time = 0
time_run = 1  # 7
start_gen = 1
greedy = False
delta = 10
per2cont = 90
low_cr = 0.005
high_cr = 0.01
par_num = 1
lar_con = 1500
start_time = start_time / (3600. * 24)
base_folder = os.getcwd()

for i in range(30):
    os.chdir(base_folder)
    c = Process(target=clock, args=((time_run - start_time) * 3600 * 24,))
    c.start()
    tic = time()
    opt = Optimization(num_gens=gen_num, greedy_allocation=greedy, allocation_delta=delta, run_time=time_run - start_time,
                       large_concept=lar_con, percent2continue=per2cont, low_cr_treshhold=low_cr,
                       high_cr_treshhold=high_cr, parents_number=par_num, gen_start=start_gen)
    try:
        opt.run()
    finally:
        opt.finish()
        print(time() - tic)
        c.terminate()
        os.rename(os.getcwd(), os.getcwd() + "_" + str(i))
