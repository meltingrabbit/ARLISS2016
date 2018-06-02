
$(function() {


	console.log("hoge");
	console.log(1<2);
	console.log("PIC112"<"PIC421");
	console.log(num2picName(1));
	console.log(num2picName(0));
	console.log(num2picName(121));
	console.log(num2picName(14421));
	var tempArr = [];
	var tempArrr = [1,2,3];
	tempArr.push(tempArrr);
	tempArr.push(tempArrr);
	console.log(tempArr);
	console.log(tempArr[1][2]);
	console.log(tempArrr);

	// ==========================================================================
	// 初期化
	// ==========================================================================

	// グローバル変数
	var strPicDataFile = 'PicData.csv';
	var inputFiles = [];
	var inputCsvFile = "";
	var inputImageFiles = [];
	var inputCsvData = [];					// PicDataClass配列


	// キャンパスの要素を取得する
	var canvas = document.getElementById( 'MapCanvas' );
	// 中心の位置座標を指定する
	var latlng = new google.maps.LatLng( 35.792621 , 139.806513 );
	// 地図のオプションを設定する
	var mapOptions = {
		zoom: 15,				// ズーム値
		center: latlng,			// 中心座標 [latlng]
	};
	// [canvas]に、[mapOptions]の内容の、地図のインスタンス([map])を作成する
	var map = new google.maps.Map( canvas , mapOptions );


	// pic data class
	function PicDataClass(time, name, gpeTime, lat, lng, height, magDeg, accX, accY, accZ) {
		//  コンストラクタ
		//  メンバ変数へ値を格納
		this.time = time;
		this.name = name;
		this.gpeTime = gpeTime;
		this.lat = lat;
		this.lng = lng;
		this.height = height;
		this.magDeg = magDeg;
		this.accX = accX;
		this.accY = accY;
		this.accZ = accZ;
	}
	//  インスタンスメソッドの定義
	PicDataClass.prototype = {

		getTime: function() {
			return( this.time );
		},
		getName: function() {
			return( this.name );
		},
		getGpeTime: function() {
			return( this.gpeTime );
		},
		getLat: function() {
			return( this.lat );
		},
		getLng: function() {
			return( this.lng );
		},
		getHeight: function() {
			return( this.height );
		},
		getMagDeg: function() {
			return( this.magDeg );
		},
		getAccX: function() {
			return( this.accX );
		},
		getAccY: function() {
			return( this.accY );
		},
		getAccZ: function() {
			return( this.accZ );
		}

	}


	// ==========================================================================
	// データファイル選択 読み込み
	// ==========================================================================

	http://www.html5rocks.com/ja/tutorials/file/dndfiles/

	function handleFileSelect(evt) {
		inputFiles = [];
		inputCsvFile = "";
		inputImageFiles = [];

		evt.stopPropagation();
		evt.preventDefault();

		inputFiles = evt.dataTransfer.files; // FileList object

		// inputFiles は File objects配列．適当にプロパティをリスト化
		var isCsvExist = 0;
		for (var i = 0; i<inputFiles.length; i++) {
			if (inputFiles[i].name == strPicDataFile) {
				inputCsvFile = inputFiles[i];
				isCsvExist = 1;
			}
			if (/^PIC.....\.JPG$/.test(inputFiles[i].name)) {
				inputImageFiles.push(inputFiles[i]);
			}
		}

		if (isCsvExist == 0) {
			alert("有効なデータファイルを選択してください．");
			return;
		}

		console.log(inputCsvFile.name);

		// PICのソート
		inputImageFiles.sort(function(a,b){
			if( a.name > b.name ) return 1;
			if( a.name < b.name ) return -1;
			return 0;
		});
		for (var i = 0; i<inputImageFiles.length; i++) {
			console.log(inputImageFiles[i].name);
		}



		var output = [];
		output.push('<p><strong>', escape(inputCsvFile.name), '</strong> (', inputCsvFile.type || 'n/a', ') - ',
		  inputCsvFile.size, ' bytes, last modified: ',
		  inputCsvFile.lastModifiedDate.toLocaleDateString(), '</p>');
		output.push('<textarea id="inputCsvFilePlaneTextArea" rows="10" cols="50" readonly></textarea>');
		for (var i = 0; i<inputImageFiles.length; i++) {
			output.push('<p><strong>', escape(inputImageFiles[i].name), '</strong> (', inputImageFiles[i].type || 'n/a', ') - ',
			  inputImageFiles[i].size, ' bytes, last modified: ',
			  inputImageFiles[i].lastModifiedDate.toLocaleDateString(), '</p>');
			output.push('<img src="" class="', inputImageFiles[i].name.slice(0,8) , '">');
		}


		document.getElementById('DropFileList').innerHTML = output.join('');

		// テキスト
		// 読み込み
		var readerCsvText = new FileReader();
		readerCsvText.readAsText(inputCsvFile);

		// 読み込み後
		readerCsvText.onload = function  () {
			// $('#OutputTxt').html(readerCsvText.result);
			// alert(readerCsvText.result);
			document.querySelector('#inputCsvFilePlaneTextArea').value = readerCsvText.result;

			parseInputCsv(readerCsvText.result);
			// JS非同期実行ゆえ，ここに置かなくてはならない．
			setListDiv();
		};

		// // 画像
		// // 読み込み
		// var reader = new FileReader();
		// reader.readAsDataURL(inputImageFiles[0]);

		// // 読み込み後
		// reader.onload = function() {
		// 	document.querySelector('#Preview').src = reader.result;
		// };


		// 画像
		var readerImage = [];
		for (var i = 0; i<inputImageFiles.length; i++) {
			// 読み込み
			readerImage.push(new FileReader());
			readerImage[i].readAsDataURL(inputImageFiles[i]);
			// 読み込み後
			// var str = inputImageFiles[i].name.slice(0,8);
			readerImage[i].classStr = inputImageFiles[i].name.slice(0,8);
			readerImage[i].onload = function() {
				$('#DropFileList img.' + this.classStr).attr('src', this.result);
			};
		}



		// var output = [];
		// for (var i = 0; i<inputFiles.length; i++) {
		// 	output.push('<li><strong>', escape(inputFiles[i].name), '</strong> (', inputFiles[i].type || 'n/a', ') - ',
		// 	  inputFiles[i].size, ' bytes, last modified: ',
		// 	  inputFiles[i].lastModifiedDate.toLocaleDateString(), '</li>');
		// }
		// document.getElementById('DropFileList').innerHTML = '<ul>' + output.join('') + '</ul>';

		// readInputFiles();

	}

	function handleDragOver(evt) {
		evt.stopPropagation();
		evt.preventDefault();
		evt.dataTransfer.dropEffect = 'copy'; // Explicitly show this is a copy
	}

	// dnd listenerの登録（イベントリスナー）
	var dropZone = document.getElementById('DropZone');
	dropZone.addEventListener('dragover', handleDragOver, false);
	dropZone.addEventListener('drop', handleFileSelect, false);

	// テスト ファイルのプレビューなど．ファイル読み込み
	// http://qiita.com/junya/items/0f977ae7f8f620fd83e7
	function readInputFiles() {

		// // 画像
		// // 読み込み
		// var reader = new FileReader();
		// reader.readAsDataURL(inputFiles[0]);

		// // 読み込み後
		// reader.onload = function  () {
		// 	document.querySelector('#Preview').src = reader.result;
		// };

		// テキスト
		// 読み込み
		var reader = new FileReader();
		reader.readAsText(inputFiles[0]);


		// 読み込み後
		reader.onload = function() {
			$('#OutputTxt').html(reader.result);
			// alert(reader.result);
			document.querySelector('#TextArea').value = reader.result;
		};
	}


	function num2picName(num) {
		var strNum = ("00000"+num).slice(-5)
		return "PIC" + strNum + ".JPG";
	}

	// input csv の parse
	function parseInputCsv(str) {
		console.log(str);
		inputCsvData = [];
		var lines = str.split("\n");
		for (var i = 0; i<lines.length; i++) {
			console.log(lines[i]);
			var lineArr = lines[i].split(",");
			var picData = new PicDataClass(+lineArr[0], lineArr[1], +lineArr[2], +lineArr[3], +lineArr[4], +lineArr[5], +lineArr[6], +lineArr[7], +lineArr[8], +lineArr[9]);
			inputCsvData.push(picData);
		}

		for (var i = 0; i<inputCsvData.length; i++) {
			console.log(inputCsvData[i]);
		}


	}

	// 右のリストテーブルを埋める
	function setListDiv() {
		var trs = [];
		for (var i = 0; i<inputCsvData.length; i++) {
			// trs.push('<tr class="' + inputCsvData[i].getName().slice(0,8) + '"><td class="time"><a href="">' + inputCsvData[i].getTime() + '</a></td><td><a href="">' + inputCsvData[i].getName() + '</a></td></tr>');
			// trs.push('<a href="#" class="button hvr-underline-from-center"><tr class="' + inputCsvData[i].getName().slice(0,8) + '"><td class="time">' + inputCsvData[i].getTime() + '</td><td>' + inputCsvData[i].getName() + '</td></tr></a>');
			trs.push('<tr class="button hvr-underline-from-center ' + inputCsvData[i].getName().slice(0,8) + '"><td class="time">' + inputCsvData[i].getTime() + '</td><td>' + inputCsvData[i].getName() + '&nbsp;&nbsp;</td></tr>');
		}
		// document.getElementById('DropFileList').innerHTML = output.join('');
		// $('#ListDiv').html('<table>' + trs.join('') + '</table>');
		// $('#ListDiv>table').html(trs.join(''));
		// $('#ListDiv>table>tbody').html('');
		$('#ListDiv>table>tbody').html(trs.join(''));
	}


	// ==========================================================================
	// 右のリストテーブルのイベント処理
	// ==========================================================================

	// ファイルを読み込んでからテーブルが作成されるので，後から追加した要素にもイベントを実行させるLiveで（documentにイベント設定）
	// 動的に生成されたボダンは親要素から指定する
	$('#ListDiv>table>tbody').on('click', 'tr', function() {
		var classStr = $(this).attr('class');
		alert("oshitane at " + classStr);
	});



});

