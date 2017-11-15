const electron = require("electron")
const url = require("url")
const path = require("path")

const {app, BrowserWindow, Menu} = electron;

//HTML windows
let main_window;

app.on("ready", function(){

    //Create new window
    main_window = new BrowserWindow({});

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

    //Build Menubar from template
    const main_menu = Menu.buildFromTemplate(mainMenuTemplate);
    Menu.setApplicationMenu(main_menu);
});

/*-----Menubar Template-----*/
const mainMenuTemplate = [
    {
        //Debug Label
        label: "Debug",
        submenu: [
            //Open Debug Tools Label
            {
                label: "Toggle Element Inspector",
                accelerator: process.platform == "Darwin" ? "Command+D": "Ctrl+D",
                click(){
                    main_window.toggleDevTools();
                }
            },
            {
                label: "Quit",
                accelerator: process.platform == "Darwin" ? "Command+Q" : "Ctrl+Q",
                click(){
                    app.quit();
                }
            }
        ]
    }
];

if(process.platform == "Darwin"){
    mainMenuTemplate.unshift({});
}