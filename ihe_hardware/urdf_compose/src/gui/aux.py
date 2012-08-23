import re
import roslib.packages

def getResolveString(path):
    directory, pkg = roslib.packages.get_dir_pkg(path)
    find = "$(find %s)" % pkg

    if directory:
        return path.replace(directory, find)

    return path

def resolvePkgPaths(string):
    # get a list of all $(find <pkg>) to resolve
    resolve = re.findall(r"\$\(find [\w.-]+\)", string)

    # get a list of packages to find
    pkgs = map(lambda x: re.sub(r"\$\(find |\)", "", x), resolve)
    paths = map(roslib.packages.get_pkg_dir, pkgs)

    for i in range(len(resolve)):
        string = string.replace(resolve[i], paths[i])

    return string

def bold(string):
    string = "<b>" + string + "</b>"

    return string

def setMargins(string, top, right, bottom, left):
    string = "<div style=\"margin:%d %d %d %d\">" + string + "</div>"
    string = string % (top, right, bottom, left)

    return string
