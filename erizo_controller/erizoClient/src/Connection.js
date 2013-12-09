/*global window, console, navigator*/

var Erizo = Erizo || {};

Erizo.sessionId = 103;

Erizo.Connection = function (spec) {
    "use strict";
    var that = {};

    spec.session_id = (Erizo.sessionId += 1);

    // Check which WebRTC Stack is installed.
    that.browser = "";
    if (typeof module !== 'undefined' && module.exports) {
	    console.log("FcStack");
        L.Logger.error('Publish/subscribe video/audio streams not supported in erizofc yet');
        that = Erizo.FcStack(spec);
    } else if (BrowserDetect.browser == "Chrome" && (parseInt(BrowserDetect.version)>=25 && parseInt(BrowserDetect.version)<=32)) {
        // Google Chrome Stable.
        console.log("Stable! "+parseInt(BrowserDetect.version));
        that = Erizo.ChromeStableStack(spec);
        that.browser = "chrome-stable";
    } else if (BrowserDetect.browser == "Chrome" && (parseInt(BrowserDetect.version)>32)) {
        // Google Chrome Canary.
        console.log("Canary! "+parseInt(BrowserDetect.version));
        that = Erizo.ChromeCanaryStack(spec);
        that.browser = "chrome-canary";
    } else if (BrowserDetect.browser == "Bowser" && (parseInt(BrowserDetect.version) == "25"|| parseInt(BrowserDetect.version)== "28")) {
        // Bowser
        that.browser = "bowser";
    } else if (BrowserDetect.browser == "Firefox" && (parseInt(BrowserDetect.version) >= 20 || parseInt(BrowserDetect.version) < 23)) {
        // Firefox
		console.log("Firefox!");
		that = Erizo.FirefoxStack(spec);
        that.browser = "mozilla";
    } else {
        // None.
        that.browser = "none";
        throw "WebRTC stack not available";
    }
    
    return that;
};

Erizo.GetUserMedia = function (config, callback, error) {
    "use strict";

    navigator.getMedia = ( navigator.getUserMedia ||
                       navigator.webkitGetUserMedia ||
                       navigator.mozGetUserMedia ||
                       navigator.msGetUserMedia);

    if (typeof module !== 'undefined' && module.exports) {
        L.Logger.error('Video/audio streams not supported in erizofc yet');
    } else {
        navigator.getMedia(config, callback, error);
    }
};
