from multiprocessing import Pool
from time import time
from tqdm import tqdm
from Other import *


def f(x):
    z = 1
    for i in range(1,x+1):
        z *= i
    return z, i


if __name__ == '__main__':
    # d = range(1, 5000)
    # p = Pool(4)
    # tic = time()
    # a = list(tqdm(p.imap(f, d), total=len(d)))
    # p.close()
    # toc = time()
    # print(toc-tic)
    # all_data = MyCsv.read_csv("results_all", "dict")
    all_concepts = load_json("jsons/concepts+configs+results")
    ga_concepts = load_json("jsons/concepts2ga")
    ga_data = {}
    for i in ga_concepts:
        if i in all_concepts:
            ga_data[i] = all_concepts[i]
