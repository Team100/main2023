# main2023

# ARCHIVED

Thanks for the memories 2023.

<hr>

[![CI](https://github.com/Team100/main2023/actions/workflows/main.yml/badge.svg)](https://github.com/Team100/main2023/actions/workflows/main.yml)

[![CI](https://github.com/Team100/main2023/actions/workflows/vision.yml/badge.svg)](https://github.com/Team100/main2023/actions/workflows/vision.yml)

Pull requests to main only, do not commit directly here.

CI follows [these directions.](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/robot-code-ci.html)

## Branches

it's not really easy to add branches to your fork.  if you really need to, you could do the recipe below, but please don't, talk to Joel instead.

```
$ git remote add upstream https://github.com/team100/main2023.git
$ git remote -v
$ git pull --all
Fetching origin
Fetching upstream
From https://github.com/team100/main2023
 * [new branch]      calgames2023 -> upstream/calgames2023
 * [new branch]      main         -> upstream/main
 * [new tag]         AfterSVR     -> AfterSVR
 * [new tag]         showmode     -> showmode
Already up to date.
$ git checkout -b calgames2023

```
