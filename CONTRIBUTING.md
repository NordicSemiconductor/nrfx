# Contributing

Thank you for considering helping us with nrfx development!

To make development clear and consistent, we have set up a few rules.
Please get familiar with them before creating pull requests. It will help us to merge your changes faster.

1. Check nrfx's [git workflow](#git-workflow).
2. [Update](#updating-the-changelog) the CHANGELOG.md file if needed.
3. Make sure your commit messages meet the [rules](#commit-message-rules).

## Git workflow

The nrfx project follows a simple git workflow. New feature branches are branched from the master
branch on a forked repository and merged back to the master branch using pull requests. History is linear.

### Working on branches
Before merging a branch back to the master branch, a few requirements must be met.

 * Make sure that CLA (Contributor License Agreement) has been accepted.
 * Make commits clear and consistent. Make sure that each commit is well-described and contains
 one logical change (see [Commit message rules](#commit-message-rules) for more). If there are
 more commits related to one logical change, squash them.
 * Rebase the branch at the top of the master branch to be mergeable. Upmerges are prohibited.
 * Avoid new commits for changes originating from reviews. If possible, use the interactive rebase
 function to introduce these changes into the existing commits.

## Updating the changelog

The changelog file was created to track significant changes in the nrfx project.

Remember that this file is created for developers. Make it simple and clear.

### Adding entries

The changelog file is split into two main sections:
 * A Version section that contains the release version and date.
 * A Changes section that is connected to the Version section.

Changes sections are strictly connected to a release version. New change descriptions must be added
in one of the four sections:
 * Added - if a new feature was added.
 * Changed - if code was updated or API was changed.
 * Fixed - if a bug was fixed.
 * Removed - if a function or code functionality was removed.

Every entry should be written in the past tense.

### Example

Every change must be placed in an appropriate section in the unreleased version. Only code owners can create
releases and assign version numbers to them.

```
# Changelog
All notable changes to this project are documented in this file.

## [Unreleased]
### Added
- Added function for checking if a TIMER instance is enabled.

### Changed
- Refactored SPIM driver and HAL.

### Fixed
- Fixed definitions related to compare channels in the TIMER HAL.

## [1.2.3] - 2020-02-20
```
### References
* http://keepachangelog.com/

## Commit message rules

To keep the history of the nrfx project clear and consistent for current and future developers, the following commit
message rules are introduced.

### Why?
Keeping commit messages clear and consistent makes history easier to browse and understand for everyone.
It can also speed up the review process and make it clearer for reviewers.

### The rules

 * Each commit should contain **one logical change**. What is a logical change?
 For example, adding a new feature or fixing a specific bug. Usually, if you cannot describe the change
 in just a few words, it is too complex for a single commit.
 To split such a complex change into several commits you can use the `git add -p` or `git add -i` commands.
 * A good commit message should answer three questions:
   * **Why is it necessary?** It may fix a bug, it may add a feature, it may improve performance,
   reliabilty, stability, or just be a change for the sake of correctness.
   * **How does it address the issue?** For short obvious patches, this part can be omitted, but it
   should be a high level description of what the approach was.
   * **What effects does the patch have?** In addition to the obvious ones, this may include
   benchmarks, side effects, and similar.
 * A commit message should contain a **short subject message**, **one blank line** and
 **a description of the change**. More details can be found at:
 https://chris.beams.io/posts/git-commit/#seven-rules .
   * **Short subject message** - short description, subject, topic of done work written in
   imperative mood (spoken or written as if giving a command or instruction, see
   [https://chris.beams.io/posts/git-commit/#imperative](https://chris.beams.io/posts/git-commit/#imperative)).
   Please start the subject line with a prefix that identifies the subsystem being changed, followed by a colon.
   Examples of good commit messages:
     *  nrfx_spi: Refactor SPI driver for readability
     *  boards: Add support for pca10040 board
     *  nrf_spi: Remove deprecated functions from SPI hal
     *  nrfx_spim: Update SPIM driver documentation
     *  nrfx_spi: Add multiple transfer functionality to SPI driver
   * **One blank line** - a blank line to separate the topic of a commit from its body.
   * **A description of the change** - The body, main description of the commit.
   Explanation of what has been changed in the code and why. Lines should be wrapped to 72
   characters. In case of a minor or obvious change, this point can be omitted.

Example of a good commit message, according to the above rules. This example is
a nonsense commit, but shows how to properly construct a good and clear commit message:

```
nrfx_spi: Remove deprecated functions from SPI driver

Remove nrfx_spi_xfer, nrfx_spi_start_task_get,
nrfx_spi_end_event_get functions and related static functions.
nrfx_spi_transfer function must be used instead of
nrfx_spi_xfer. nrfx_spi_xfer was removed according to decision
to not support legacy API.

nrfx_spi_start_task_get and nrfx_spi_end_event_get
functionalities were moved to nrfx_common module to support more
generic API and are not supported in this version.

A side effect of these changes is speeding up interrupt handlers
when using two serial peripherals. Benchmark results are as follows:
 - Two peripherals, 8 B, xfer function code - 250 us
 - Two peripherals, 8 B, transter function code - 190 us
```
### References
* http://who-t.blogspot.no/2009/12/on-commit-messages.html
* https://chris.beams.io/posts/git-commit
