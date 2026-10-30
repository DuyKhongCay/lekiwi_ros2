#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "chess_episode_recorder/campaign_quota_domain.hpp"
#include "chess_episode_recorder/dataset_storage.hpp"

using namespace chess_episode_recorder::storage;
using namespace chess_episode_recorder::domain;

class DatasetStorageTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_dir_ = std::filesystem::temp_directory_path() / "lekiwi_test_dataset_storage";
    std::filesystem::remove_all(test_dir_);
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(test_dir_);
  }

  std::filesystem::path test_dir_;
};

TEST_F(DatasetStorageTest, Scanner_ScanNextEpisodeIndex)
{
  EXPECT_EQ(DatasetDirectoryScanner::scan_next_episode_index(test_dir_), 0u);

  std::filesystem::create_directories(test_dir_ / "episode_000000");
  std::filesystem::create_directories(test_dir_ / "episode_000001");
  std::filesystem::create_directories(test_dir_ / "episode_000005");

  EXPECT_EQ(DatasetDirectoryScanner::scan_next_episode_index(test_dir_), 6u);
}

TEST_F(DatasetStorageTest, Scanner_MakeEpisodeDir)
{
  auto path = DatasetDirectoryScanner::make_episode_dir(test_dir_, 42);
  EXPECT_EQ(path.filename().string(), "episode_000042");
  EXPECT_EQ(path.parent_path(), test_dir_);
}

TEST_F(DatasetStorageTest, Scanner_WriteMetadataAndScanRecover)
{
  auto ep0 = test_dir_ / "episode_000000";
  std::filesystem::create_directories(ep0);

  EpisodeMetadata meta;
  meta.episode_id = 0;
  meta.task = "Move white knight from b1 to c3";
  meta.duration_sec = 12.5;
  meta.timestamp_start = 100.0;
  meta.timestamp_end = 112.5;
  meta.message_counts["/camera/wrist/image_raw/compressed"] = 350;

  EXPECT_TRUE(DatasetDirectoryScanner::write_episode_info(ep0, meta));
  EXPECT_TRUE(std::filesystem::exists(ep0 / "episode_info.yaml"));

  // Add an invalid episode directory to test quarantine
  auto ep1 = test_dir_ / "episode_000001";
  std::filesystem::create_directories(ep1);
  {
    std::ofstream fout(ep1 / "episode_info.yaml");
    fout << "Corrupted invalid YAML: [ : } \n";
  }

  auto report = DatasetDirectoryScanner::scan_and_recover(test_dir_, true);
  EXPECT_EQ(report.recovered_episodes, 1u);
  ASSERT_EQ(report.recovered_instructions.size(), 1u);
  EXPECT_EQ(report.recovered_instructions[0], meta.task);
  EXPECT_EQ(report.quarantined_dirs.size(), 1u);
  EXPECT_TRUE(std::filesystem::exists(test_dir_ / "quarantine" / "episode_000001"));
}

TEST_F(DatasetStorageTest, SessionStateYamlRepository_SaveAndLoad)
{
  auto yaml_path = test_dir_ / "session_state.yaml";

  SessionState state;
  state.session_id = 7;
  state.episodes_in_session = 14;
  state.initial_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";
  state.active_fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1";
  state.is_active = true;
  state.executed_moves = {"e2e4"};

  EXPECT_TRUE(SessionStateYamlRepository::save(yaml_path, state));
  EXPECT_TRUE(std::filesystem::exists(yaml_path));

  auto loaded = SessionStateYamlRepository::load(yaml_path);
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->session_id, 7u);
  EXPECT_EQ(loaded->episodes_in_session, 14u);
  EXPECT_EQ(loaded->initial_fen, state.initial_fen);
  EXPECT_EQ(loaded->active_fen, state.active_fen);
  EXPECT_TRUE(loaded->is_active);
  ASSERT_EQ(loaded->executed_moves.size(), 1u);
  EXPECT_EQ(loaded->executed_moves[0], "e2e4");
}

TEST_F(DatasetStorageTest, DiskSpace_Query)
{
  auto info = query_disk_space(test_dir_);
  EXPECT_TRUE(info.success);
  EXPECT_GT(info.available_bytes, 0u);
}

TEST_F(DatasetStorageTest, McapEpisodeWriter_Lifecycle)
{
  McapEpisodeWriter writer("mcap");
  EXPECT_FALSE(writer.is_open());

  auto ep_dir = test_dir_ / "episode_000099";
  std::map<std::string, std::string> topics = {
    {"/cmd_vel", "geometry_msgs/msg/Twist"}
  };

  EXPECT_TRUE(writer.open_episode(ep_dir, 99, "test_exp", "test task", topics, "cdr"));
  EXPECT_TRUE(writer.is_open());

  writer.close();
  EXPECT_FALSE(writer.is_open());
}

