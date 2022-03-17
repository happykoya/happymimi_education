#!/usr/bin/env python
#-*- coding: utf-8 -*-


def main():
    result = speechRecog(short_str=True).result_str
    rsu = str(result)
    print(rsu)
    result2 = speechRecog(short_str=True).result_str
    rsu2 = str(result2)
    print(rsu2)

    print(Levenshtein.distance(rsu,rsu2))

if __name__ == '__main__':
    main()
