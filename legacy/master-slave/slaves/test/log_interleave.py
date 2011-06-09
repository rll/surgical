import sys, re

def interleave(left_file, right_file, output_file):
    """Interleave logfile triplets such that each
    triplet occured before all following triplets"""

    left    = left_file.readlines()
    right   = right_file.readlines()
    all = left + right
    all.sort(logfile_sort)

    for triple in all:
        output_file.write(triple)

def logfile_sort(left, right):
    """Sort logfiles where each line starts with:
            <slave_num> <unix_time> <target_pitch>...
    """

    expr    =   re.compile("^[12]\s+(\d+).+")

    time_left   =   int(expr.search(left).group(1))
    time_right  =   int(expr.search(right).group(1))

    if time_left <= time_right:
        return -1
    else:
        return 1

if __name__ == "__main__":
    assert len(sys.argv) == 4

    left_file   = open(sys.argv[1], 'r')
    right_file  = open(sys.argv[2], 'r')
    output_file = open(sys.argv[3], 'w')

    interleave(left_file, right_file, output_file)

    left_file.close()
    right_file.close()
    output_file.close()

