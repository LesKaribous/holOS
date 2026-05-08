"""Virtual-camera server — reads a video / image / USB camera and serves
the frames as MJPEG-over-HTTP. No pipeline, no analysis, just the raw
stream + a small UI to drive playback. Runs alongside holOS and the
pipeline debug runner; both consume the same `/stream.mjpg`."""
