from Other import MyCsv
from tqdm import tqdm
import numpy as np


all_data = MyCsv.read_csv("results_all", "dict")  # all the results
confs = []
for dat in tqdm(all_data):
    if dat["name"] != "name":
        confs.append(dat["name"])

confs2 = np.asarray(confs)
a = np.unique(confs2)
a = np.ndarray.tolist(a)
b = []
for i in tqdm(confs):
    if i not in a:
        b.append(i)
    else:
        a.remove(i)

