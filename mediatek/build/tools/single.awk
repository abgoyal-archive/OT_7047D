#! /usr/bin/awk -f
#-------------------------------------------------------------------------------
# Purpose:     get non-repeat word
# Author:      Jibin Zhang
# Created:     2013-06-20
# update:      2013-06-28
#-------------------------------------------------------------------------------
{
    for (i = 1; i <= NF; i++)
    {
        ++word[$i]
        if (word[$i] == 1)
            printf("%s ", $i)
    }
}
# chmod u+x t.awk
# ./t.awk file1

