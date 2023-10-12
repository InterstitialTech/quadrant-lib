# Packaging/Releasing

To package this library into a zip file, first make sure your have no un-committed
changes, and that your current commit is tagged with the version number. Then:
```
cd util/
bash release.sh
```

This will create a zip file in the working directory. To publish a release to
Github, first,
```
git push --tags origin main
```

Then create a new release, and add the zip file as an assett.

