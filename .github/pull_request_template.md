Before submitting a pull request, make sure you satisfy **all** of the following points:

- [ ] Give as much detail as possible on what is included within the pull request
- [ ] Make sure that new additions are not already present in the project
- [ ] If it contains source code, comply with the [coding guidelines](https://github.com/juanmanzanero/lion-cpp/blob/main/CONTRIBUTING.md)
- [ ] Add new tests for new features
- [ ] Run and pass the tests using GitHub's workflows

If you are submitting a circuit, you must submit:
- [ ] The google earth KML files as `database/tracks/track-name/track-name-left.kml` and `database/tracks/track-name/track-name-right.kml`
- [ ] The XML file created using the circuit preprocessor as `database/tracks/track-name/track-name.xml`
- [ ] [This jupyter notebook](https://github.com/juanmanzanero/fastest-lap/tree/main/examples/python/circuit_preprocessor) applied to your track that includes the circuit preprocessor call and the track representation as `database/tracks/track-name/track-name.ipynb`
- [ ] The notebook converted to Markdown as `README.md`.
