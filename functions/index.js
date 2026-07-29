const { onValueWritten } = require("firebase-functions/v2/database");
const { initializeApp } = require("firebase-admin/app");
const { getFirestore, FieldValue } = require("firebase-admin/firestore");

initializeApp();

// Cấu hình rõ instance Database nằm ở server asia-southeast1
exports.saveLocationHistory = onValueWritten(
    {
        ref: "/vehicle",
        instance: "gps-tracking-a01d3-default-rtdb",
    },
    async (event) => {
        const data = event.data.after.val();

        if (!data || data.latitude === undefined || data.longitude === undefined) {
            return;
        }

        const db = getFirestore();

        try {
            await db.collection("location_history").add({
                latitude: data.latitude,
                longitude: data.longitude,
                status: data.status || "UNKNOWN",
                timestamp: FieldValue.serverTimestamp(),
            });
            console.log(`✅ Đã lưu lịch sử: ${data.latitude}, ${data.longitude}`);
        } catch (error) {
            console.error("❌ Lỗi ghi Firestore:", error);
        }
    }
);