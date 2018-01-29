var ls;
(function (ls) {
	ls.playScene = function() {
		return {
			"%22backOut%22": function() { return "backOut" },
			"myBird.y%2B%3D6": function() { return myBird.y+=6 },
			"ls.random()*15%2B0.8": function() { return ls.random()*15+0.8 },
			"-(600-AISprite2920.y)": function() { return -(600-AISprite2920.y) },
			"AISprite2925": function() { return AISprite2925 },
			"%22equalTo%22": function() { return "equalTo" },
			"%22overScene%22": function() { return "overScene" },
			"null": function() { return null },
			"myBird.y-55": function() { return myBird.y-55 },
			"%22totalNum%22": function() { return "totalNum" },
			"%22tween%22": function() { return "tween" },
			"AISprite187": function() { return AISprite187 },
			"AISprite2920": function() { return AISprite2920 },
			"ls.random()*200%2B620": function() { return ls.random()*200+620 }
		}
	};
	ls.MainScene = function() {
		return {
			"%22tween1%22": function() { return "tween1" },
			"%22playScene%22": function() { return "playScene" },
			"%22bounceOut%22": function() { return "bounceOut" }
		}
	};
	ls.overScene = function() {
		return {
			"System.totalNum": function() { return System.totalNum },
			"%22playScene%22": function() { return "playScene" }
		}
	};
})(ls || (ls = {}));