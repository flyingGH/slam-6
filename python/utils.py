import time
from contextlib import contextmanager


@contextmanager
def timer(name: str):
    """
    時刻測定

    :param name: 測定の description
    """
    t0 = time.time()
    yield
    print(f'[{name}]: {time.time() - t0:.3f} s')
