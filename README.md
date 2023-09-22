# quadrant-lib
## Arduino Library for Quadrant

# Installing
Download the latest zip file from the GitHub releases, and follow the directions
[here](https://support.arduino.cc/hc/en-us/articles/5145457742236-Add-libraries-to-Arduino-IDE).

# Packaging/Releasing
To package this library into a zip file, first make sure your have no un-committed
changes, and that the your current commit is tagged with the version number. Then:
```
cd util/
bash release.sh
``
This will create a zip file in the working directory, which you can upload as an
assett to a new GitHub release.
