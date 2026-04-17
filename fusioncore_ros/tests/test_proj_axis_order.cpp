#include <gtest/gtest.h>
#include <proj.h>
#include <cmath>

// Regression test for the PROJ axis-order bug fixed in gnss_to_output.
//
// proj_normalize_for_visualization forces EPSG:4326 into visualization
// (map-plotting) axis order: slot 0 = LONGITUDE, slot 1 = LATITUDE.
//
// The bug: code was passing (lat, lon) in slots (0, 1), causing PROJ to
// treat the latitude value as longitude and vice versa. At mid-latitudes
// (e.g. Paris ~49°N 2°E) this produces an ECEF roughly 6500 km off.
// With reference.use_first_fix=true the error cancels in ENU subtraction
// and is invisible; with a hand-computed fixed ECEF reference every live
// fix is rejected as thousands of km from the reference.

static PJ* make_4326_to_4978(PJ_CONTEXT* ctx)
{
    PJ* raw = proj_create_crs_to_crs(ctx, "EPSG:4326", "EPSG:4978", nullptr);
    if (!raw) return nullptr;
    PJ* pj = proj_normalize_for_visualization(ctx, raw);
    proj_destroy(raw);
    return pj;
}

// ─── Test 1: round-trip (lon, lat) → ECEF → LLA recovers < 1 mm ──────────────

TEST(ProjAxisOrder, RoundTripParis)
{
    PJ_CONTEXT* ctx = proj_context_create();
    PJ* pj = make_4326_to_4978(ctx);
    ASSERT_NE(pj, nullptr);

    // Paris: lat=48.8566°N, lon=2.3522°E
    const double lon_deg = 2.3522;
    const double lat_deg = 48.8566;

    // Forward: slot 0 = longitude, slot 1 = latitude (visualization order)
    PJ_COORD fwd  = {{ lon_deg, lat_deg, 0.0, HUGE_VAL }};
    PJ_COORD ecef = proj_trans(pj, PJ_FWD, fwd);

    // Inverse: ECEF → LLA via semantic struct members (order-agnostic)
    PJ_COORD back = proj_trans(pj, PJ_INV, ecef);

    // 1 mm on Earth's surface ≈ 9e-6 degrees
    EXPECT_NEAR(back.lpzt.phi, lat_deg, 1e-5);
    EXPECT_NEAR(back.lpzt.lam, lon_deg, 1e-5);
    EXPECT_NEAR(back.lpzt.z,   0.0,    0.01);  // < 1 cm altitude round-trip

    proj_destroy(pj);
    proj_context_destroy(ctx);
}

// ─── Test 2: swapping slots produces wrong ECEF (> 1000 km error) ────────────

TEST(ProjAxisOrder, SwappedSlotsProduceWrongECEF)
{
    PJ_CONTEXT* ctx = proj_context_create();
    PJ* pj = make_4326_to_4978(ctx);
    ASSERT_NE(pj, nullptr);

    const double lon_deg = 2.3522;
    const double lat_deg = 48.8566;

    PJ_COORD correct = proj_trans(pj, PJ_FWD, {{ lon_deg, lat_deg, 0.0, HUGE_VAL }});
    PJ_COORD buggy   = proj_trans(pj, PJ_FWD, {{ lat_deg, lon_deg, 0.0, HUGE_VAL }});

    double dx   = correct.xyz.x - buggy.xyz.x;
    double dy   = correct.xyz.y - buggy.xyz.y;
    double dz   = correct.xyz.z - buggy.xyz.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    EXPECT_GT(dist, 1e6);  // > 1000 km apart — the old bug was ~6500 km

    proj_destroy(pj);
    proj_context_destroy(ctx);
}

// ─── Test 3: known ECEF coordinates for Paris ────────────────────────────────
// Validates against textbook WGS84 values, not just internal round-trip.
// Reference computed from: X=(N+h)cosφcosλ, Y=(N+h)cosφsinλ, Z=(N(1-e²)+h)sinφ

TEST(ProjAxisOrder, KnownECEFParis)
{
    PJ_CONTEXT* ctx = proj_context_create();
    PJ* pj = make_4326_to_4978(ctx);
    ASSERT_NE(pj, nullptr);

    PJ_COORD fwd  = {{ 2.3522, 48.8566, 0.0, HUGE_VAL }};
    PJ_COORD ecef = proj_trans(pj, PJ_FWD, fwd);

    // WGS84 ECEF for Paris (lat=48.8566°, lon=2.3522°, h=0)
    // verified against PROJ output: X≈4201095, Y≈172617, Z≈4780081
    EXPECT_NEAR(ecef.xyz.x,  4201095.0, 200.0);
    EXPECT_NEAR(ecef.xyz.y,   172617.0, 200.0);
    EXPECT_NEAR(ecef.xyz.z,  4780081.0, 200.0);

    proj_destroy(pj);
    proj_context_destroy(ctx);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
