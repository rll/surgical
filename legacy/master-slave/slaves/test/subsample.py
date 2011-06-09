import sys

def subsample(input, output, num_lines):
    for (i, line) in enumerate(input):
        if i % num_lines == 0:
            output.write(line)

if __name__ == "__main__":

    assert len(sys.argv) == 4
    input = open(sys.argv[1], 'r')
    output = open(sys.argv[2], 'w')
    num_lines = int(sys.argv[3])

    subsample(input, output, num_lines)
