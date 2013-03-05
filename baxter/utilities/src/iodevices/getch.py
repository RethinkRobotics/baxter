import sys
import termios
import tty
from select import select

def getch(timeout=0.01):
    """ retrieves a character from stdin
    Returns None if no character is available within the timeout
    Blocks if timeout < 0
    """
    fileno = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fileno)
    ch = None
    try:
        tty.setraw(fileno)
        rlist = [fileno]
        if timeout >= 0:
            [rlist, _, _] = select(rlist, [], [], timeout)
        if fileno in rlist:
            ch = sys.stdin.read(1)
    except Exception as ex:
        print "getch", ex
        raise OSError
    finally:
        termios.tcsetattr(fileno, termios.TCSADRAIN, old_settings)
    return ch

if __name__ == '__main__':
    print("getch test")
    print("blocking getch...")
    c = getch(-1)
    print("returned %s" % (str(c),))
    print("2.5 second timeout getch...")
    c = getch(2.5)
    print("returned %s" % (str(c),))
    print("non-blocking getch in loop until character...")
    while not getch():
        pass
    print("done.")
