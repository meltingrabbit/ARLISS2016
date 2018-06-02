
$(function() {

	// ==========================================================================
	// 初期化
	// ==========================================================================

	// グローバル変数
	var strPathNull = 'PicData.csvが選択されていません．';
	$('div#Path>p.path').html(strPathNull);
	var strPicDataFile = 'PicData.csv';


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
	function PicDataClass(time, name, gpeTime, lat, lng, height, magDeg, accX, accY, accZ){
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
	// PicData.csv選択 読み込み
	// ==========================================================================

	// $('button#GetPath').on('click', function() {
	// 	alert("oshita");

	// $.when(
	// 	// $('input#GetPathInput').trigger('click')
	// 	hoge()
	// ).done(function() {
	// 	alert("done")
	// });
	// });

	$('button#GetPath').on('click', function() {
		// alert("oshita");
		// $('input#GetPathInput').val('');
		// var str = $('input#GetPathInput').val();
		// alert(str);

		$('input#GetPathInput').trigger('click');
		// onClick=$('input#GetPathInput').click();

		// var FileInfo = new Array();
		// var FilePath = "";
		// FileInfo = document.all.PicDataCsv.value.split('\\');
		// for (i = 0; i < FileInfo.length - 2; i++) {
		// 		FilePath = FilePath + FileInfo[i] + '\\';
		// }
		// FilePath += FileInfo[FileInfo.length - 2];

		// alert("ファイル＝" + document.all.PicDataCsv.value + "\n  パス＝" + FilePath);
	});

	$('input#GetPathInput').change(function() {
		// alert("henko");
		// var str = $('input#GetPathInput').val();
		// alert(str);

		var fileInfo = new Array();
		var filePath = "";
		fileInfo = document.all.PicDataCsv.value.split('\\');
		for (i = 0; i < fileInfo.length - 2; i++) {
				filePath = filePath + fileInfo[i] + '\\';
		}
		fileName = fileInfo[fileInfo.length - 1];
		filePath += fileInfo[fileInfo.length - 2];
		// alert("ファイル＝" + document.all.PicDataCsv.value + "\n  パス＝" + filePath + "\n  名前＝" + fileName);

		if (fileName == strPicDataFile) {
			$('div#Path>p.path').html(document.all.PicDataCsv.value);
		} else {
			alert("有効なデータファイルを選択してください．");
			$('div#Path>p.path').html(strPathNull);
			return;
		}
		alert("ok");
	});




	function hoge() {
		$('input#GetPathInput').trigger('click')
		alert("click")
		return 1;
	}

});

