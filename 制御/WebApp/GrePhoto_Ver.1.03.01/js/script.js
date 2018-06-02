
$(function() {


	// console.log("hoge");
	// console.log(1<2);
	// console.log("PIC112"<"PIC421");
	// console.log(num2picName(1));
	// console.log(num2picName(0));
	// console.log(num2picName(121));
	// console.log(num2picName(14421));
	// var tempArr = [];
	// var tempArrr = [1,2,3];
	// tempArr.push(tempArrr);
	// tempArr.push(tempArrr);
	// console.log(tempArr);
	// console.log(tempArr[1][2]);
	// console.log(tempArrr);

	// ==========================================================================
	// 初期化
	// ==========================================================================

	// グローバル変数
	var strPicDataFile = 'PicData.csv';
	var inputFiles = [];
	var inputCsvFile = "";
	var inputImageFiles = [];
	var inputCsvData = [];					// PicDataClass配列
	var markers = {};						// マーカーの連想配列
	var lines = [];						// 線の配列
	var markerEventListeners = [];

	var bounds;



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

		// console.log(inputCsvFile.name);

		// PICのソート
		inputImageFiles.sort(function(a,b) {
			if( a.name > b.name ) return 1;
			if( a.name < b.name ) return -1;
			return 0;
		});
		// for (var i = 0; i<inputImageFiles.length; i++) {
		// 	console.log(inputImageFiles[i].name);
		// }


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
			unsetMarkerEvent();
			setMarker();
			setLine();
			setMarkerEvent();
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

	// input csv の parse
	function parseInputCsv(str) {
		// console.log(str);
		inputCsvData = [];
		var lines = str.split("\n");
		// 最終行は読み込まない
		for (var i = 0; i<(lines.length-1); i++) {
			// console.log(lines[i]);
			var lineArr = lines[i].split(",");
			var picData = new PicDataClass(lineArr[0], lineArr[1], lineArr[2], lineArr[3], lineArr[4], lineArr[5], lineArr[6], lineArr[7], lineArr[8], lineArr[9]);
			inputCsvData.push(picData);
		}

		for (var i = 0; i<inputCsvData.length; i++) {
			console.log(inputCsvData[i]);
		}
	}

	// 右のリストテーブルと下の詳細情報（クリックしてdisplay:block）を埋める
	function setListDiv() {
		var trs = [];
		var picInfo = [];
		for (var i = 0; i<inputCsvData.length; i++) {
			// trs.push('<tr class="' + inputCsvData[i].getName().slice(0,8) + '"><td class="time"><a href="">' + inputCsvData[i].getTime() + '</a></td><td><a href="">' + inputCsvData[i].getName() + '</a></td></tr>');
			// trs.push('<a href="#" class="button hvr-underline-from-center"><tr class="' + inputCsvData[i].getName().slice(0,8) + '"><td class="time">' + inputCsvData[i].getTime() + '</td><td>' + inputCsvData[i].getName() + '</td></tr></a>');
			// trs.push('<tr class="button hvr-underline-from-center ' + inputCsvData[i].getName().slice(0,8) + '"><td class="time">' + inputCsvData[i].getTime() + '</td><td>' + inputCsvData[i].getName() + '&nbsp;&nbsp;</td></tr>');

			trs.push('<tr id="ListDiv' + inputCsvData[i].getName().slice(0,8) + '" class="list-div-tr ' + inputCsvData[i].getName().slice(0,8) + '"><td><div class="clearfix button"><div class="left time">' + inputCsvData[i].getTime() + '</div><div class="right pic-name">' + inputCsvData[i].getName() + '</div></div>');
			trs.push('<div class="clearfix hover-info">');
			trs.push('<div class="left img"><img src="./img/NoImage.png" class="' + inputCsvData[i].getName().slice(0,8) + '"></div><div class="right info"><table><tbody>');
			trs.push('<tr><td class="key">GPSt</td><td>' + inputCsvData[i].getGpeTime() + '</td></tr>');
			trs.push('<tr><td class="key">Lat</td><td>' + inputCsvData[i].getLat() + '</td></tr>');
			trs.push('<tr><td class="key">Lng</td><td>' + inputCsvData[i].getLng() + '</td></tr>');
			trs.push('<tr><td class="key">Hght</td><td>' + inputCsvData[i].getHeight() + '</td></tr>');
			trs.push('<tr><td class="key">Mag</td><td>' + inputCsvData[i].getMagDeg() + '</td></tr>');
			trs.push('<tr><td class="key">AccX</td><td>' + inputCsvData[i].getAccX() + '</td></tr>');
			trs.push('<tr><td class="key">AccY</td><td>' + inputCsvData[i].getAccY() + '</td></tr>');
			trs.push('<tr><td class="key">AccZ</td><td>' + inputCsvData[i].getAccZ() + '</td></tr></tbody></table></div></div></td></tr>');

			picInfo.push('<div class="pic-info clearfix ' + inputCsvData[i].getName().slice(0,8) + '" style="display:none;"><div class="pic-info-img left"><img src="./img/NoImage.png"></div><div class="pic-info-info right"><table><tbody>');
			picInfo.push('<tr><td class="key"><strong>PIC name</strong></td><td class="info">&nbsp;&nbsp;&nbsp;' + inputCsvData[i].getName() + '</td></tr>');
			picInfo.push('<tr><td class="key"><strong>GPS time</strong></td><td class="info">&nbsp;&nbsp;&nbsp;' + inputCsvData[i].getGpeTime() + '</td></tr>');
			picInfo.push('<tr><td class="key"><strong>Lat</strong></td><td class="info">&nbsp;&nbsp;&nbsp;' + inputCsvData[i].getLat() + '</td></tr>');
			picInfo.push('<tr><td class="key"><strong>Lng</strong></td><td class="info">&nbsp;&nbsp;&nbsp;' + inputCsvData[i].getLng() + '</td></tr>');
			picInfo.push('<tr><td class="key"><strong>Height</strong></td><td class="info">&nbsp;&nbsp;&nbsp;' + inputCsvData[i].getHeight() + '</td></tr>');
			picInfo.push('<tr><td class="key"><strong>Mag Deg</strong></td><td class="info">&nbsp;&nbsp;&nbsp;' + inputCsvData[i].getMagDeg() + '</td></tr>');
			picInfo.push('<tr><td class="key"><strong>Acc X</strong></td><td class="info">&nbsp;&nbsp;&nbsp;' + inputCsvData[i].getAccX() + '</td></tr>');
			picInfo.push('<tr><td class="key"><strong>Acc Y</strong></td><td class="info">&nbsp;&nbsp;&nbsp;' + inputCsvData[i].getAccY() + '</td></tr>');
			picInfo.push('<tr><td class="key"><strong>Acc Z</strong></td><td class="info">&nbsp;&nbsp;&nbsp;' + inputCsvData[i].getAccZ() + '</td></tr>');
			picInfo.push('</tbody></table></div></div>');

		}
		// document.getElementById('DropFileList').innerHTML = output.join('');
		// $('#ListDiv').html('<table>' + trs.join('') + '</table>');
		// $('#ListDiv>table').html(trs.join(''));
		// $('#ListDiv>table>tbody').html('');
		$('#ListDiv>table>tbody').html(trs.join(''));
		$('#Sub40').html(picInfo.join(''));

		// 存在する画像は挿入
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
				$('#ListDiv div.hover-info img.' + this.classStr).attr('src', this.result);
				$('#Sub40 div.' + this.classStr + ' img').attr('src', this.result);
			};
		}
	}

	// マップにマーカーを設置
	function setMarker() {
		// 古いマーカーの削除
		for(key in markers) {
			markers[key].setMap(null);
		}
		markers = {};
		var lats = [];
		var lngs = [];
		// テスト
		// new google.maps.Marker( {
		// 	map: map,
		// 	position: latlng,
		// });

		for (var i = 0; i<inputCsvData.length; i++) {
			// マーカーのインスタンス化
			markers[ inputCsvData[i].getName() ] = new google.maps.Marker( {
				map: map,
				position: new google.maps.LatLng(inputCsvData[i].getLat() , inputCsvData[i].getLng()),
			});
			lats.push(inputCsvData[i].getLat());
			lngs.push(inputCsvData[i].getLng());
		}

		//北東端の座標を設定
		var ne = new google.maps.LatLng(Math.max.apply(null, lats), Math.max.apply(null, lngs));
		//南西端の座標を設定
		var sw = new google.maps.LatLng(Math.min.apply(null, lats), Math.min.apply(null, lngs));
		//範囲を設定
		bounds = new google.maps.LatLngBounds(sw, ne);
		//マーカーが全て収まるように地図の中心とズームを調整して表示
		map.fitBounds(bounds);
	}

	// マップに線を設置
	function setLine() {
		// 古い線の削除
		for (var i = 0; i<lines.length; i++) {
			lines[i].setMap(null);
		}
		lines = [];

		for (var i = 0; i<(inputCsvData.length-1); i++) {
			// 線のインスタンス化
			lines[i] = new google.maps.Polyline( {
				map: map,
				path: [
					new google.maps.LatLng(inputCsvData[i].getLat(), inputCsvData[i].getLng()),
					new google.maps.LatLng(inputCsvData[i+1].getLat(), inputCsvData[i+1].getLng()),
				],
			});
		}
	}


	// ==========================================================================
	// イベント処理
	// ==========================================================================

	// ファイルを読み込んでからテーブルが作成されるので，後から追加した要素にもイベントを実行させるLiveで（documentにイベント設定）
	// 動的に生成された要素は親要素から指定する
	$('#ListDiv>table>tbody').on('click', 'tr.list-div-tr', function() {
		var classStr = $(this).attr('class');
		classStr = classStr.replace('list-div-tr ' ,'');
		// alert("oshitane at " + classStr);
		$('#Sub40>div').css('display', 'none');
		$('#Sub40>div.' + classStr).css('display', 'block');

		// 地図中心の移動
		map.setCenter(markers[classStr + '.JPG'].getPosition());
		// alert(markers[classStr + '.JPG'].getPosition());
		// map.setCenter(new google.maps.LatLng( 35.792621 , 139.806513 ));
	});

	// マーカーのイベントの設定
	// クロージャー内包
	function setMarkerEvent() {
		for(key in markers) {
			onMarkerClick(markers[key], key);
			onMarkerMouseover(markers[key], key);
			onMarkerMouseout(markers[key], key);
		}
	}

	// マーカークリック時のイベント
	// クロージャー
	function onMarkerClick(marker, picName) {
		var listener = google.maps.event.addListener(
			marker,
			'click',
			function(event){
				$('#Sub40>div').css('display', 'none');
				$('#Sub40>div.' + picName.slice(0,8)).css('display', 'block');
			}
		);
		markerEventListeners.push(listener);
	}

	// マーカーホバーin時のイベント
	// クロージャー
	function onMarkerMouseover(marker, picName) {
		var listener = google.maps.event.addListener(
			marker,
			'mouseover',
			function(event){
				$('#ListDiv').stop();			// 前のアニメーション停止
				hoverIn('ListDiv' + picName.slice(0,8));
				// // 現在の縦スクロール位置
				// var scrollPosition = document.getElementById('ListDiv').scrollTop;
				// // スクロール要素の高さ
				// var scrollHeight = document.getElementById('ListDiv').scrollHeight;
				// 何番目か？
				var index = $('#ListDiv' + picName.slice(0,8)).index();
				// alert(picName.slice(0,8) + '\n' + scrollPosition + '\n' + scrollHeight + '\n' + index);
				// document.getElementById('ListDiv').scrollTop = 24 * index;				// アニメーションなしVer.

				// アニメーションありVer.
				var speed = 250;
				var position = 24 * index;
				$('#ListDiv').animate({scrollTop:position}, speed, "swing");
				return false;
			}
		);
		markerEventListeners.push(listener);
	}

	// マーカーホバーout時のイベント
	// クロージャー
	function onMarkerMouseout(marker, picName) {
		var listener = google.maps.event.addListener(
			marker,
			'mouseout',
			function(event){
				hoverOut()
			}
		);
		markerEventListeners.push(listener);
	}

	// イベントリスナーの削除
	function unsetMarkerEvent() {
		for (var i = 0; i<markerEventListeners.length; i++) {
			google.maps.event.removeListener( markerEventListeners[i] );
		}
		markerEventListeners = [];
	}

	function num2picName(num) {
		var strNum = ("00000"+num).slice(-5)
		return "PIC" + strNum + ".JPG";
	}

	// ホバーアクション
	// 動的に生成された要素は親要素から指定する
	$('#ListDiv>table>tbody').on({
	'mouseenter': function() {
		hoverIn($(this).attr("id"));
	},
	'mouseleave': function() {
		hoverOut();
	}
	}, 'tr.list-div-tr');

	// ホバーin
	function hoverIn(id) {
		// alert($(this).attr("id"));
		$('#ListDiv>table>tbody div.hover-info').css('display', 'none');
		$('#' + id + ' div.hover-info').css('display', 'block');
	}

	// ホバーout
	function hoverOut() {
		// alert('hovr not');
		$('#ListDiv>table>tbody div.hover-info').css('display', 'none');
	}

	$('#ResetButton').on("click", function() {

		map.fitBounds(bounds);
	});
});

