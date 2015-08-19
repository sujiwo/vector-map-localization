#!/usr/bin/python2

lst = [0, 1, 2, 3, 4]
listlen = len(lst)

for i in range(listlen/2) :
# 	ps = lst[i]
# 	pt = lst[listlen-1-i]
# 	print (pt)
	tmp = lst[i]
	lst [i] = lst[listlen-1-i]
	lst[listlen-1-i] = tmp
print (lst)
