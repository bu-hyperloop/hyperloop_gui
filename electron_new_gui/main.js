const electron = require("electron")
const url = require("url")
const path = require("path")

const {app, BrowserWindow} = electron;

let main_window;

app.on("ready", function(){

    //Create new window
    main_window = new BrowserWindow({
        frame : false
    });

    //Load HTML into window
    main_window.loadURL(url.format({
        pathname: path.join(__dirname, "main_window.html"),
        protocol: "file",
        slashes: true
    }));

    //Handler when "main_window" is closed
    main_window.on("closed", function(){
        app.quit();
    });
});