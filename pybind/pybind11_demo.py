from lib import mylibs


def main():
    print(mylibs.add(3, 5))

    p = mylibs.POINT(3, 5)
    print(p.X(), p.Y(), p.sum)

    q = mylibs.move_p(p, 10)
    print(q.X(), q.Y(), q.sum)

    print(mylibs.vec_double([1, 2, 4, 5]))

    print(mylibs.vec_add([
        [1, 1, 1],
        [1, 2, 3, 4, 5],
        [1, -1, 1, -1, 1, -1],
        [1, 2, 4, 8, 16, 32, 64]
    ]))


if __name__ == "__main__":
    main()
