function fixHeight() {
    var bodyh = document.getElementById('mainBody').offsetHeight; /* change this */
    var navh = document.getElementById('navSide').offsetHeight;
    var conth = document.getElementById('mainContent').offsetHeight;
    if(bodyh > navh)
    {
		document.getElementById('mainContent').style.height = bodyh + 'px';
		document.getElementById('navSide').style.height = bodyh + 'px';
	}
}
fixHeight();
