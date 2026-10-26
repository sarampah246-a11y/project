#script by @Massatriof16 - Thanks @Massatriof16 for script
current_dir=/tmp
kernel_dir=$(pwd)
cd $current_dir
echo " Checking clang instelled clang "

if [ -e $current_dir/clang ]; then
echo " Clang sudah ada, setting enviroment clang"
armm=$current_dir/arm-linux-androideabi-4.9-12-5/bin/arm-linux-androideabi-
clang=$current_dir/clang/bin
export PATH="$(pwd)/clang/bin:$(pwd)/aarch64-linux-android-4.9-12-5/bin:${PATH}"
else
echo "clang Tidak ada! Mendownload clang terlebih dahulu"
mkdir -p $current_dir/clang
cd $current_dir/clang
wget https://android.googlesource.com/platform/prebuilts/clang/host/linux-x86/+archive/refs/tags/android-12.0.0_r12/clang-r416183b.tar.gz
tar xf clang-r416183b.tar.gz
cd $current_dir
git clone https://github.com/cooked71/arm-linux-androideabi-4.9-12-5
armm=$current_dir/arm-linux-androideabi-4.9-12-5/bin/arm-linux-androideabi-
clang=$current_dir/clang/bin
export PATH="$(pwd)/clang/bin:$(pwd)/aarch64-linux-android-4.9-12-5/bin:${PATH}"
fi

echo " Checking swap File... "
if [ -e $current_dir/swapfile ]; then
echo " file swap sudah ada !"
else
echo " swap tidak ada, membuat file swap"
sudo fallocate -l 30G swapfile
sudo dd if=/dev/zero of=swapfile bs=1M count=30720
sudo chmod 600 swapfile
sudo mkswap swapfile
sudo swapon swapfile
echo "vm.swappiness=100" | sudo tee -a /etc/sysctl.conf
fi

cd $kernel_dir

if [ ! -e $kernel_dir/out ]; then
make aether_itel_a70_defconfig O=out \
            ARCH=arm64 \
            LLVM=1 \
            CC=clang\
            AR=$clang/llvm-ar \
            AS=$clang/llvm-as \
            NM=$clang/llvm-nm \
            OBJCOPY=$clang/llvm-objcopy \
            OBJDUMP=$clang/llvm-objdump \
            STRIP=$clang/llvm-strip \
            CLANG_TRIPLE=aarch64-linux-gnu- \
            CROSS_COMPILE_COMPAT="$armm"\
            CROSS_COMPILE=aarch64-linux-android-
fi
                      
make O=out \
            ARCH=arm64 \
            LLVM=1 \
            CC=clang\
            AR=$clang/llvm-ar \
            AS=$clang/llvm-as \
            NM=$clang/llvm-nm \
            OBJCOPY=$clang/llvm-objcopy \
            OBJDUMP=$clang/llvm-objdump \
            STRIP=$clang/llvm-strip \
            CLANG_TRIPLE=aarch64-linux-gnu- \
            CROSS_COMPILE_COMPAT="$armm" \
            CROSS_COMPILE=aarch64-linux-android-
