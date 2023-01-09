### Table of Contents
- [Contributing](#contributing)
    + [How to Install Repo on a new Computer:](#how-to-install-repo-on-a-new-computer)
    + [Any time you want to make a change:](#any-time-you-want-to-make-a-change)
- [Tips for Code Review](#tips-for-code-review)
    + [Comment on 1 or more lines of code:](#comment-on-1-or-more-lines-of-code)
    + [Finishing Code Reviews](#finishing-code-reviews)

## Contributing


### How to Install Repo on a new Computer:


Clone this repository

```
$ git clone https://github.com/Team3256/FRC_Programming_2023.git
```

Install all necessary tools: Gradle, Third-Party libraries, etc.

```
$ ./gradlew
```
Open the project in IntelliJ and make the edits you want!

### Any time you want to make a change:
We are currently using Github Flow, which can be read about [here](https://guides.github.com/introduction/flow/).

1. Create and checkout a new branch
    - `git checkout -b <your_branch_name>`, where <your_branch_name> is a descriptive name for your branch. For example `fix/fix-shooter-wheel`, `feat/two-ball-auto`, or `feat/climbing`. Use dashes in the branch name, not underscores. Preface the branch name with either `fix/` or `feat/`.
2. Make any code changes you need/want to make.
    - Try to make your code as readable as possible, remember you're not just writing code for yourself, but everyone else too. A good tool for documenting your code is [JavaDocs]T(https://www.jetbrains.com/help/idea/working-with-code-documentation.html).
    - You can use comments to clarify confusing aspects of your code that aren't super readable.
3. Commit your work
    - Make sure to write [meaningful commit messages](https://chris.beams.io/posts/git-commit/) so that we can easily see what changes have been made.
4. Make a Pull Request of your Branch
    - Once you finish your code, create a pull request and [ask some people](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/requesting-a-pull-request-review) to review your code.
5. Wait for Others to Review your Code
    -  It might take some time for others to review your code, so feel free to work on something else in the meantime.
    - If you make any changes you can ask reviewers to re-review your code.
    - **Note:** In order to Merge your branch with `master` you need to have at least 2 reviewers approve of your changes.
6. Merge your Pull Request
    - If there are no conflicts select [Squash and Merge](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/about-pull-request-merges#squash-and-merge-your-pull-request-commits), write a good commit message and merge the changes.
    - If there are conflicts, change them locally on your branch, push them, and then hit Squash and Merge.

## Tips for Code Review

When you review someone else's code, make sure you can __understand what they changed__ and __why__.

If something does not seem necessary or you don't know what it does, make sure to __ask__ in the pull request.

### Comment on 1 or more lines of code:
1. On the Files Changed tab of the pull request, navigate to the file you want to comment on.
2. Select the blue plus (+) next to the line or hold and drag on the plus to select multiple lines.
3. Either Start a review or if you've already started a review, add a comment to your review.
4. When done adding comments select either Approve, Comment, or Request Changes to finish your review.

### Finishing Code Reviews

* **Request Changes** will __block the pull request__, until the same reviewer approves of the changes. The best use case for this is something that is critical to the functionality of the program or something that cripples readability.

* **Comment** will only __add a comment and does not deny or approve the request__, these should be used for changes that might not be entirely required but can help add to the code.(eg. formatting)

* **Approve** will indicate that you see __nothing wrong and the changes should be merged__ into the main branch.
