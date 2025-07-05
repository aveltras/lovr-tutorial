# Usage

You must have a `.env` file in your project directory with the following content.

Install [Direnv](https://direnv.net/) to automatically load all the required dependencies or just `nix develop` if you don't want direnv.

```
LOVR_DIRECTORY=PATH_TO_YOUR_LOCAL_LOVR_CLONE_HERE
ANDROID_KEYSTORE=
ANDROID_KEYSTORE_PASS=
```
This template uses [Just](https://github.com/casey/just) to manage its available commands.

You can see available commands using `just` in your terminal.

```sh
Available recipes:
    apk-build buildtype="Release"
    apk-install
    build
    default
    keystore-generate name
    prepare-build-directory directory
```
Running `just build` command will compile LÖVR from the path configured with the `LOVR_DIRECTORY` environment variable set in the `.env` file.

The `lovr` binary is then available in `build/bin` (automatically added to your path if you use direnv).
