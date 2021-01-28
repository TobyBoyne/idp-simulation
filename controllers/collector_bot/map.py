import matplotlib.pyplot as plt

x = []
y = []

# open file and read the content in a list
with open('listfile.txt', 'r') as filehandle:
    for line in filehandle:
        # remove linebreak which is the last character of the string
        currentPlace = line[:-1]
        m = currentPlace.split(', ')
        X = float(m[0][1:])
        Y = float(m[1][:-1])

        # add item to the list
        x.append(X)
        y.append(Y)

plt.scatter(x, y)
