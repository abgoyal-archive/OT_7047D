1. cd /Your_SourceCode
  for example :
	cd /alps

2. export PATH=/Your_Toolchain_PATH/:$PATH
   
   for example :
	export PATH=/alps/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.6/bin:$PATH

3. Build Command:
   ./makeMtk -t yaris_xl n k
