function fixHeight() {
    var bodyh = document.getElementById('mainBody').offsetHeight; /* change this */
    var navh = document.getElementById('navSide').offsetHeight;
    var conth = document.getElementById('mainContent').offsetHeight;
    console.error("bodyh:",bodyh);
    console.error("navh:",navh);
    if(bodyh > navh)
    {
		document.getElementById('mainContent').style.height = document.getElementById('rst-content').offsetHeight+'px';
		document.getElementById('navSide').style.height = document.getElementById('rst-content').offsetHeight+'px';
   }
}
fixHeight();
