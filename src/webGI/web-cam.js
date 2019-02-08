'use strict';

// streaming only video
const mediaStreamConstraints = {
  video: true,
};

// element where stream will be placed
const localVideo = document.querySelector('video');

// stream that will be reproduced on the video
let localStream;

function gotLocalMediaStream(mediaStream) {
  localStream = mediaStream;
  localVideo.srcObject = mediaStream;
}

// error handling
function handleLocalMediaStreamError(error) {
  console.log('navigator.getUserMedia error: ', error);
}

// initialization of media stream
navigator.mediaDevices.getUserMedia(mediaStreamConstraints)
  .then(gotLocalMediaStream).catch(handleLocalMediaStreamError);
