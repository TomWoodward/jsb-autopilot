# JP's notes

Run this in VSCode, using the provided devcontainer.

`yarn jest --watch`

`rollup -c` to build a minified version, then strip off the "use strict" at the front and the export at the end.  Copy remainder into a tank script.

`yard add blah --dev` when new packages needed.

# Nathan's notes

* install libraries with: `yarn install`
* test code by running in browser: `yarn start`
  * stimulation will open in your default browser and restart after every code change
  * future improvement will be to store selected competitor in localStorage and re-use it after reload
* Add your code to `main.js`, after saving it the browser should reload so you can view results immediatly
* Build code for copy'n'paste to competition: `yarn build`
* Code will be built ininto a `dist/index.<hash>.js` file, copy from it

# Usage

Copy the `bundle/bundle.min.js` into your JS Battle tank script.  Remove

```js
"use strict";Object.defineProperty(exports,"__esModule",{value:!0});
```

from the front and remove

```js
exports.Autopilot=Autopilot,exports.Constants=Constants;
```

from the end after you paste it in.

See `bundle/bundle.js` for a non-minified version. Then in your tank script (below where you pasted the bundle), say:

```js
let autopilot = new Autopilot();

tank.loop(function(state, control) {
	autopilot.update(state, control);

  // your code here
```

The `Autopilot` object gives you some utility functions to help you build your tank:

* After your radar has scanned a horizontal and vertical wall, the autopilot will have calculated the battlefield origin for you.  You can access it with `autopilot.origin`.  `autopilot.isOriginKnown()` will return true when the origin is available.
* If you know the origin through other means (e.g. from your teammates), you can set it with `setOrigin(x,y)`.
* `lookEverywhere` sets your radar to spin at max speed.
* `lookAtEnemy(enemy)` locks your radar onto an enemy.  `enemy` is an object with `x` and `y` coordinates.
* `isWallCollisionImminent(inTicks=3)` returns true if it looks like your tank will hit a wall in the number of ticks provided.
* `turnToAngle(angle)` turns your bot to the given absolute angle.  Returns the angle remaining to turn.
* `turnToPoint(x,y,baseOnZeroOrigin=false)` will turn your tank to face the given coordinates.  If the origin is known and you set `baseOnZeroOrigin` to true, you can give coordinates assuming the origin is (0,0). Returns the angle remaining to turn.
* `moveToPoint(x,y,baseOnZeroOrigin=false)` will move your bot to the provided coordinates.  If the origin is known and you set `baseOnZeroOrigin` to true, you can give coordinates assuming the origin is (0,0).
* `moveAlongAngle(angle)` moves your tank in the given direction.  You can use constants here, e.g. `moveAlongAngle(Constants.EAST)`.
* `loopOnPath(positions, basedOnZeroOrigin, tolerance=50)` cycles through an array of `{x:, y:}` positions, which can be based on zero origin if the origin is known.  The `tolerance` says how close is close enough to say we've made it to one of the positions.
* `stopLoopOnPath()` clears a `loopOnPath` call.
* `stop()` stops the turn, throttle, and boost.
* `shootEnemy()` shoots the enemy (enemy object from the `state`), using a predictive aim based on the enemy's speed and angle.
* `extrapolatedPosition(inTicks)` gives your bot's linear extrapolated position in the given number of ticks.  May not totally account for speed that is ramping up or slowing down.
* `extrapolatedOuterPosition(inTicks)` is the same as the above except it uses a circle around the tank to give the position (to help with collision detection)
* `speed()` gives the tank's speed.
* `Autopilot.extrapolatedPosition(startPosition, travelAngle, travelSpeed, inTicks)` computes a general linear extrapolated position.




# javascript-library-template [![GitHub license](https://img.shields.io/github/license/vvo/javascript-library-template?style=flat)](https://github.com/vvo/javascript-library-template/blob/master/LICENSE) [![Tests](https://github.com/vvo/javascript-library-template/workflows/CI/badge.svg)](https://github.com/vvo/javascript-library-template/actions) [![codecov](https://codecov.io/gh/vvo/javascript-library-template/branch/master/graph/badge.svg)](https://codecov.io/gh/vvo/javascript-library-template) ![npm](https://img.shields.io/npm/v/javascript-library-template) [![minizipped size](https://badgen.net/bundlephobia/minzip/javascript-library-template)](https://bundlephobia.com/result?p=javascript-library-template)

<p align="center">
<small><b>Click below to create a new GitHub repository using this template:</b></small>
<br/><br/><a href="https://github.com/vvo/javascript-library-template/generate">
<img src="https://img.shields.io/badge/use%20this-template-blue?logo=github">
</a>
</p>

---

**This JavaScript library template** allows you to easily develop, collaborate on and publish a JavaScript library with all the modern tooling you'd expect from the current JavaScript ecosystem.

**Why should you use this?** One of the hidden challenges of authoring opensource JavaScript libraries is to provide a project that is easy to contribute to. You want people to join your project. Doing so requires a good amount of boilerplate: testing, code coverage, dependencies maintenance, release scripts, tooling requirements (Node.js, Yarn and which versions are we using again?), code editor configuration, formatting, linting... Well, this is the goal of this template: **to provide sensible and modern defaults to all those subjects**. So that once set up, you can focus on ?????? coding, ???? collaborating and ???? shipping.

**The goals of the template are to:**

- Ease the contribution of the library by providing reproducible environments for developers and CI
- Automate as much as possible, from testing to releasing and upgrading dependencies
- Provide good defaults for users of [Visual Studio Code](https://code.visualstudio.com/)

**Features:**

- [EditorConfig](https://editorconfig.org/): easy contributions from any code editor.
- [ESLint](https://eslint.org/): launched in the `test` script.
- [Prettier](https://prettier.io/): launched in the `test` script, with markdown, JavaScript, CSS and JSON files support (including automatic [`package.json` formatting](https://github.com/matzkoh/prettier-plugin-packagejson)).
- Automatic VSCode formatting and linting: using VSCode extensions recommendations and workspace settings in .`vscode/` folder.
- [Yarn](http://yarnpkg.com/) version pinning: via [Yarn policies](https://classic.yarnpkg.com/en/docs/cli/policies/), so anyone contributing or any system accessing your library will use the same Yarn version without having to think about it.
- [Node.js](https://nodejs.org/) version pinning: via [nvm](https://github.com/nvm-sh/nvm), so anyone contributing or any system accessing your library will use the same Node.js version without having to think about it.
- [Jest](https://jestjs.io/): launched in the `test` script, also with the right VSCode settings providing a testing workflow inside VSCode using [`vscode-jest`](https://github.com/jest-community/vscode-jest) extension.
- [GitHub actions](https://github.com/features/actions): automatic testing and releasing from GitHub: npm publish and GitHub releases are automatically created. Note that the package.json in your repository is never updated (the version is always `0.0.0-development`), only the one in npm is updated. This is surprising at first but as long as you display the published version in your README (like this template does) then you're fine. Find more information about this in the [semantic-release documentation](https://semantic-release.gitbook.io/semantic-release/support/faq#why-is-the-package-jsons-version-not-updated-in-my-repository).
- [semantic-release](https://semantic-release.gitbook.io/semantic-release/): allows for automatic releases based on semver.org and [conventional commits specification](https://www.conventionalcommits.org/). The defaults are taken from the [Angular git commit guidelines](https://github.com/angular/angular.js/blob/master/DEVELOPERS.md#-git-commit-guidelines).
- [Codecov](https://codecov.io/): launched in the `test` script on CI, ensures code coverage does not decrease on pull requests _(free for public and private repositories)_.
- [Renovate](https://renovate.whitesourcesoftware.com/) configurated with the [JavaScript library preset](https://docs.renovatebot.com/presets-config/#configjs-lib): this will automatically update your dependencies by opening pull request for you to approve or not. So you never have to think about it _(free for public and private repositories)_.

## Setup

Using this template requires a bit of setup, but way less than if you had to start from 0. Here's what you need to do:

**Required steps:** (needed every time you want to use the template)

1. [Create a new repository](https://github.com/new) on GitHub based on this template
1. [Setup renovate](https://github.com/apps/renovate) for your new repository. If you previously installed the Renovate application to your account then this is just a box to tick when creating the repository
1. [Clone the new repository](https://help.github.com/en/github/creating-cloning-and-archiving-repositories/cloning-a-repository)
1. Change the package name and description in `package.json`
1. [Setup Codecov](https://github.com/apps/codecov) for your new repository. If you previously installed the Codecov application to your account then this is just a box to tick when creating the repository
1. Setup semantic releases: run `yarn semantic-release-cli setup` in a terminal (This will ask for your npm and GitHub credentials)
1. Add the previously generated `GH_TOKEN` and `NPM_TOKEN` secrets to the [GitHub secrets](https://help.github.com/en/actions/configuring-and-managing-workflows/creating-and-storing-encrypted-secrets#creating-encrypted-secrets) of the new repository
1. Install dependencies: run `yarn` in your terminal
1. **Develop your library**: change code in `lib/`
1. **Test your library**: you can either see tests results inside VSCode directly or run `yarn jest --watch`
1. **Check formatting of your code**: run `yarn check-formatting` in your terminal
1. **Create your first release**: [open a pull request](https://help.github.com/en/desktop/contributing-to-projects/creating-a-pull-request) on your project, wait for tests to pass, merge and ???? your library will be automatically released to npm and a GitHub release will be created

**Optional steps:** (needed only if you're doing them for the first time)

1. If you're not using VSCode, if your editor has no EditorConfig support, then [setup EditorConfig](https://editorconfig.org/#download) [EditorConfig support](https://editorconfig.org/)
1. Make sure you have [npm 2fa auth-only](https://docs.npmjs.com/about-two-factor-authentication#authorization-only) configured. Releases can't be automated if you have 2fa setup for both authentication and publish. See https://semantic-release.gitbook.io/semantic-release/usage/ci-configuration#authentication-for-plugins
1. [Install nvm](https://github.com/nvm-sh/nvm)
1. [Install yarn](https://classic.yarnpkg.com/en/docs/install#alternatives-stable)

## Status and next steps

The template is still pretty new (March 2020) and was done to author JavaScript libraries using ECMAScript modules for Node.js >= 12. Gradually, or given requests, **we could update it to support**:

- Using different CI environments than GitHub actions
- Authoring browser libraries
- Generating `README.md` table of contents automatically
- Better default `README.md` content (Install, API, Examples, ...)
- `create-javascript-library` command line that would get most of the setup done easily
- `.github` Pull requests template, issues templates, CONTRIBUTING files
- add or change scripts to allow for auto-formatting
- provide documentation on how to protect branches on GitHub
- provide scripts to easily open a pull request once a branch is created
- provide a way to check for semantic commits in PR
- contributors list
- https://github.com/apps/semantic-pull-requests

If you'd like to participate, if you have bugs or new ideas, [open an issue](https://github.com/vvo/javascript-library-template/issues/new) or a pull request.

## Recipes

### Using `yarn link`

To use `yarn link` efficiently, do this:

```bash
> cd my-library
> yarn link
> yarn build --watch
> cd ../my-other-library
> yarn link my-library
```

### Reformating all code

```bash
yarn format
```
