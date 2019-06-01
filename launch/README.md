## 关于一些问题
因为对于使用动态参数配置，它的cfg文件好像修改默认值后再次启动节点并不能正确修改默认值。
但是我们需要飞机启动后就加载成为修改好的阈值等值，所以查了文档后，使用
```
<node pkg="dynamic_reconfigure" name="threshold_adjust" type="dynparam" args="set /detect_black/detect_black threshold $(arg threshold)" />
```
可以在启动黑圆检测节点时手动输入一个arg改变阈值，更好的方法后面继续探索