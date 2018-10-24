#!/usr/bin/env python3

__author__ = 'Bence Cserna'


def write(text):
    print(text, end="", flush=True)


def print_start(out):
    out.write("@>>>_#>>>>>>__")


def print_middle(out):
    out.write("""
####v########v
___<_>_#______
v#####v#_#####
_#___#_#______
v#v#^#v######_
_#_#_#_#______
v#_#_#_#_#####
_#_#_#_#_#_#__
_<<#___#___#__
v############v
v<<<<<<<<<<<<<
_>>>_>>>>>>>>v""")


def print_end(out):
    out.write("""
####v########v
___<_>_#______
v#####v#_#####
_#___#_#______
v#v#^#v######_
_#_#_#_#______
v#_#_#_#_#####
_#_#_#_#_#_#__
_<<#___#___#__
v############v
______*_______
""")


def create_highway(n):
    file = open('highways/highway_{}.vw'.format(n), 'w+')
    file.write("14\n")
    file.write("{}\n".format(1 + 11 + n * 12))
    print_start(file)
    for _ in range(0, n):
        print_middle(file)
    print_end(file)
    file.close()
    pass


def main():
    for i in range(10, 1001, 10):
        create_highway(i)


if __name__ == "__main__":
    main()
