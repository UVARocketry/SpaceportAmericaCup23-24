# SpaceportAmericaCup23-24

To get your computer setup to contribute to the code in this repo, you should get Git installed. Ya Git it?

If you don't know what Git is, what it's used for, or how to use it here are some simple tutorials:

-> https://rogerdudler.github.io/git-guide/ <- This is gold if I ever seen it

-> https://dzone.com/articles/top-20-git-commands-with-examples

Document on how to get everything installed: https://myuva-my.sharepoint.com/:w:/r/personal/dt3zjy_virginia_edu/Documents/UVA%20Rocketry%20Club/Competition/2022-23%20(BOTR)/Electronics/Getting%20IDE_s%20Set%20Up.docx?d=w27eef7ca4d4440f38b05672d9cb1cded&csf=1&web=1&e=uobfoF

## Environment

### Software

#### Github

All our code will be hosted on github, the link to the repository for this year will be shared once yall become members of the rocketry organization

#### Git

Git will be used for managing the source code. It can be downloaded for windows [here](https://gitforwindows.org/), for linux, it can be installed via package manager, on mac, the install instructions can be found [here](https://git-scm.com/download/mac)

#### Platformio

We use platformio for connecting and uploading software to the circuit board.

If you use vscode, platformio has a [vscode extension](https://platformio.org/install/ide?install=vscode).

If you use neovim, you should first install the platformio [cli](https://platformio.org/install/cli) (this can probably also be installed through your os's package manager), then there is a [lua plugin](https://github.com/anurag3301/nvim-platformio.lua) that I personally have not used yet, but it seems very promising (last year, I followed a modified version of [these instructions](https://docs.platformio.org/en/latest/integration/ide/vim.html) to get lsp support, but I will be trying out nvim-platformio to see how well it works and I might end up contributing to it to get dap support).

#### Drivers

For sensor drivers, there are a bunch of free and open source drivers published by [adafruit](https://www.adafruit.com/). Some partial usages of some of these drivers exist in our competition code from [last year](https://github.com/UVARocketry/SpaceportAmericaCup23-24/blob/main/gyrodriver/src/main.cpp).

#### RocketPy

We will probably be using [rocketpy](https://docs.rocketpy.org/en/latest/) to test our program, though this might change later in the year. You dont have to worry too much about this for now as this wont become important until we start getting our guidance code working

### Hardware

#### Sensors

To order sensors we will be using [digikey](https://www.digikey.com/en/products/result?s=N4IgTCBcDaIOYAcDOIC6BfIA) and [mouser](https://www.mouser.com/). Right now, the sensors we will probably end up using are a GPS, IMU, and altimeter/barometer.

#### Altium

To actually design the board, we will use [altium designer](https://www.altium.com/altium-designer?srsltid=AfmBOopCOouTD5QUP6iWCQxnMLjkHSUI74x4dRBxMl9jCRZfiPy_f8Zu).
